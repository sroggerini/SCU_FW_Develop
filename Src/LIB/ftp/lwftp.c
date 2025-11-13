/*
 * lwftp.c : a lightweight FTP client using raw API of LWIP
 *
 * Copyright (c) 2014 GEZEDO
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * Author: Laurent GONZALEZ <lwip@gezedo.com>
 *
 */
#ifdef FTP_SERVER

#include <string.h>
#include "lwftp.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "ftps.h"

#include "telnet.h"

/** Enable debugging for LWFTP */
#ifndef LWFTP_DEBUG
#define LWFTP_DEBUG   LWIP_DBG_ON
#endif

#define LWFTP_TRACE   (LWFTP_DEBUG|LWIP_DBG_TRACE)
#define LWFTP_STATE   (LWFTP_DEBUG|LWIP_DBG_STATE)
#define LWFTP_WARNING (LWFTP_DEBUG|LWIP_DBG_LEVEL_WARNING)
#define LWFTP_SERIOUS (LWFTP_DEBUG|LWIP_DBG_LEVEL_SERIOUS)
#define LWFTP_SEVERE  (LWFTP_DEBUG|LWIP_DBG_LEVEL_SEVERE)

#define PTRNLEN(s)  s,(sizeof(s)-1)

#define FILE_TO_GET   "hw.cfg"

static const char fileToRetr [] = FILE_TO_GET;
static uint32_t   fileSize;

static void lwftp_control_process(lwftp_session_t *s, struct tcp_pcb *tpcb, struct pbuf *p);

/** Close control or data pcb
 * @param pointer to lwftp session data
 */
static err_t lwftp_pcb_close(struct tcp_pcb *tpcb)
{
  err_t error;

  tcp_err(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  error = tcp_close(tpcb);
  if ( error != ERR_OK ) {
    tPrintf("lwftp:pcb close failure, not implemented\n");
  }
  return ERR_OK;
}

/** Send data
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param number of bytes sent
 */
static err_t lwftp_send_next_data(lwftp_session_t *s)
{
  const char *data;
  int len = 0;
  err_t error = ERR_OK;

  if (s->data_source) {
    len = s->data_source(s->handle, &data, s->data_pcb->mss);
    if (len) {
      error = tcp_write(s->data_pcb, data, len, 0);
      if (error!=ERR_OK) {
        tPrintf("lwftp:write failure (%s), not implemented\n",lwip_strerr(error));
      }
    }
  }
  if (!len) {
    tPrintf("lwftp:end of file\n");
    lwftp_pcb_close(s->data_pcb);
    s->data_pcb = NULL;
  }
  return ERR_OK;
}

/** Handle data connection incoming data
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param pointer to incoming pbuf
 * @param state of incoming process
 */
static err_t lwftp_data_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;
  if (p) {
    if (s->data_sink) {
      struct pbuf *q;
      tPrintf("lwftp_data_recv\n\r");
      for (q=p; q; q=q->next) {
        s->data_sink(s->handle, q->payload, q->len);
        tPrintf("lwftp_data_recv %d \n\r", q->len);
      }
    } else {
      tPrintf("lwftp: sinking %d bytes\n",p->tot_len);
    }
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
  } else {
    // NULL pbuf shall lead to close the pcb. Close is postponed after
    // the session state machine updates. No need to close right here.
    // Instead we kindly tell data sink we are done
    if (s->data_sink) {
      s->data_sink(s->handle, NULL, 0);
    }
  }
  return ERR_OK;
}

/** Handle data connection acknowledge of sent data
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param number of bytes sent
 */
static err_t lwftp_data_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if ( s->data_source ) {
    s->data_source(s->handle, NULL, len);
  }
  return lwftp_send_next_data(s);
}

/** Handle data connection error
 * @param pointer to lwftp session data
 * @param state of connection
 */
static void lwftp_data_err(void *arg, err_t err)
{
  LWIP_UNUSED_ARG(err);
  if (arg != NULL) {
    lwftp_session_t *s = (lwftp_session_t*)arg;
    tPrintf("lwftp:failed/error connecting for data to server (%s)\n",lwip_strerr(err));
    s->data_pcb = NULL; // No need to de-allocate PCB
    if (s->control_state==LWFTP_XFERING) { // gracefully move control session ahead
      s->control_state = LWFTP_DATAEND;
      lwftp_control_process(s, NULL, NULL);
    }
  }
}

/** Process newly connected PCB
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param state of connection
 */
static err_t lwftp_data_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if ( err == ERR_OK ) {
    tPrintf("lwftp:connected for data to server\n");
    s->data_state = LWFTP_CONNECTED;
  } else {
    tPrintf("lwftp:err in data_connected (%s)\n",lwip_strerr(err));
  }
  return err;
}

/** Open data connection for passive transfer
 * @param pointer to lwftp session data
 * @param pointer to incoming PASV response
 */
static err_t lwftp_data_open(lwftp_session_t *s, struct pbuf *p)
{
  err_t error;
  char *ptr;
  ip_addr_t data_server;
  u16_t data_port;

  // Find server connection parameter
  ptr = strchr(p->payload, '(');
  if (!ptr) return ERR_BUF;
  do {
    unsigned int a = strtoul(ptr+1,&ptr,10);
    unsigned int b = strtoul(ptr+1,&ptr,10);
    unsigned int c = strtoul(ptr+1,&ptr,10);
    unsigned int d = strtoul(ptr+1,&ptr,10);
    IP4_ADDR(&data_server,a,b,c,d);
  } while(0);
  data_port  = strtoul(ptr+1,&ptr,10) << 8;
  data_port |= strtoul(ptr+1,&ptr,10) & 255;
  if (*ptr!=')') return ERR_BUF;

  // Open data session
  tcp_arg(s->data_pcb, s);
  tcp_err(s->data_pcb, lwftp_data_err);
  tcp_recv(s->data_pcb, lwftp_data_recv);
  tcp_sent(s->data_pcb, lwftp_data_sent);
  error = tcp_connect(s->data_pcb, &data_server, data_port, lwftp_data_connected);
  return error;
}

/** Send a message to control connection
 * @param pointer to lwftp session data
 * @param pointer to message string
 */
static err_t lwftp_send_msg(lwftp_session_t *s, const char* msg, size_t len)
{
  err_t error;

  tPrintf("lwftp:sending %s",msg);
  error = tcp_write(s->control_pcb, msg, len, 0);
  if ( error != ERR_OK ) {
      tPrintf("lwftp:cannot write (%s)\n",lwip_strerr(error));
  }
  return error;
}

/** Close data connection
 * @param pointer to lwftp session data
 * @param result to pass to callback fn (if called)
 */
static void lwftp_data_close(lwftp_session_t *s, int result)
{
  if (s->data_pcb) {
    lwftp_pcb_close(s->data_pcb);
    s->data_pcb = NULL;
  }
  if ( s->done_fn ) {
    s->done_fn(s->handle, result);
  }
}

/** Close control connection
 * @param pointer to lwftp session data
 * @param result to pass to callback fn (if called)
 */
static void lwftp_control_close(lwftp_session_t *s, int result)
{
  if (s->data_pcb) {
    lwftp_pcb_close(s->data_pcb);
    s->data_pcb = NULL;
  }
  if (s->control_pcb) {
    lwftp_pcb_close(s->control_pcb);
    s->control_pcb = NULL;
  }
  s->control_state = LWFTP_CLOSED;
  if ( (result >= 0) && s->done_fn ) {
    s->done_fn(s->handle, result);
  }
}

/** Main client state machine
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param pointer to incoming data
 */
static void lwftp_control_process(lwftp_session_t *s, struct tcp_pcb *tpcb, struct pbuf *p)
{
  uint32_t response, len;
  int result;

  response = 0;
  result = LWFTP_RESULT_ERR_SRVR_RESP;

  // Try to get response number
  if (p) {
    response = strtoul(p->payload, NULL, 10);
    tPrintf("lwftp:got response %d\n",response);
  }

  switch (s->control_state) {
    case LWFTP_CONNECTED:
      if (response>0) {
        if (response==220) {
          lwftp_send_msg(s, PTRNLEN("USER "));
          lwftp_send_msg(s, s->user, strlen(s->user));
          lwftp_send_msg(s, PTRNLEN("\r\n"));
          s->control_state = LWFTP_USER_SENT;
        } else {
          s->control_state = LWFTP_QUIT;
        }
      }
      break;
    case LWFTP_USER_SENT:
      if (response>0) {
        if (response==331) {
          lwftp_send_msg(s, PTRNLEN("PASS "));
          lwftp_send_msg(s, s->pass, strlen(s->pass));
          lwftp_send_msg(s, PTRNLEN("\r\n"));
          s->control_state = LWFTP_PASS_SENT;
        } else {
          s->control_state = LWFTP_QUIT;
        }
      }
      break;
    case LWFTP_PASS_SENT:
      if (response>0) {
        if (response==230) {
          s->control_state = LWFTP_LOGGED;
          tPrintf("lwftp: now logged in\n");
          if (s->done_fn) 
          {
              s->done_fn(s->handle, LWFTP_RESULT_LOGGED);
          }
        } else {
          s->control_state = LWFTP_QUIT;
        }
      }
      break;
    case LWFTP_TYPE_SENT:
      if (response>0) {
        if (response==200) {
          lwftp_send_msg(s, PTRNLEN("PASV\r\n"));
          s->control_state = LWFTP_PASV_SENT;
        } else {
          s->control_state = LWFTP_QUIT;
        }
      }
      break;
    case LWFTP_PASV_SENT:
      if (response>0) {
        if (response==227) {
          lwftp_data_open(s,p);
          switch (s->target_state) 
          {
            case LWFTP_STOR_SENT:
              lwftp_send_msg(s, PTRNLEN("STOR "));
              break;
            case LWFTP_RETR_SENT:
              lwftp_send_msg(s, PTRNLEN("RETR "));
              break;
            case LWFTP_MLSD_SENT:
              lwftp_send_msg(s, PTRNLEN("LIST \r\n"));
              /* il server Microsoft presente di default su W7, non riconosce MLSD */
              //lwftp_send_msg(s, PTRNLEN("MLSD \r\n"));
              break;
            default:
              tPrintf("Unexpected internal state");
              s->target_state = LWFTP_QUIT;
          }
          len = (uint32_t)strlen(s->remote_path);
          if (len != 0)
          {
            lwftp_send_msg(s, s->remote_path, len);
            lwftp_send_msg(s, PTRNLEN("\n"));
          }
          s->control_state = s->target_state;
        } else {
          s->control_state = LWFTP_QUIT;
        }
      }
      break;
    case LWFTP_RETR_SENT:
      if (response>0) {
        if (response==150) {
          s->control_state = LWFTP_XFERING;
        } else if (response==550) {
            s->control_state = LWFTP_DATAEND;
            result = LWFTP_RESULT_ERR_FILENAME;
            tPrintf("lwftp: Failed to open file '%s'\n", s->remote_path);
        }
        else {
          s->control_state = LWFTP_DATAEND;
          tPrintf("lwftp:expected 150, received %d\n",response);
        }
      }
      break;
    case LWFTP_STOR_SENT:
      if (response>0) {
        if (response==150) {
          s->control_state = LWFTP_XFERING;
          lwftp_data_sent(s,NULL,0);
        } else {
          s->control_state = LWFTP_DATAEND;
          tPrintf("lwftp:expected 150, received %d\n",response);
        }
      }
      break;
    case LWFTP_MLSD_SENT:
      if (response>0) {
        if ((response==150) || (response==226))
        {
          if (response==226)
          {
            s->control_state = LWFTP_QUIT;
          }
        }
        else 
        {
          s->control_state = LWFTP_DATAEND;
          tPrintf("lwftp:expected 226, received %d\n",response);
        }
      }
      break;
    case LWFTP_XFERING:
      if (response>0) {
        if (response==226) {
          result = LWFTP_RESULT_OK;
        } else {
          result = LWFTP_RESULT_ERR_CLOSED;
          tPrintf("lwftp:expected 226, received %d\n",response);
        }
        s->control_state = LWFTP_DATAEND;
      }
      break;
    case LWFTP_DATAEND:
      tPrintf("forced end of data session");
      break;
    case LWFTP_QUIT_SENT:
      if (response>0) {
        if (response==221) {
          result = LWFTP_RESULT_OK;
        } else {
          result = LWFTP_RESULT_ERR_UNKNOWN;
          tPrintf("lwftp:expected 221, received %d\n",response);
        }
        s->control_state = LWFTP_CLOSING;
      }
      break;
    default:
      tPrintf("lwftp:unhandled state (%d)\n",s->control_state);
  }

  // Free receiving pbuf if any
  if (p) {
    pbuf_free(p);
  }

  // Handle second step in state machine
  switch ( s->control_state ) {
    case LWFTP_DATAEND:
      lwftp_data_close(s, result);
      s->control_state = LWFTP_LOGGED;
      break;
    case LWFTP_QUIT:
      lwftp_send_msg(s, PTRNLEN("QUIT \r\n"));
      tcp_output(s->control_pcb);
      s->control_state = LWFTP_QUIT_SENT;
      break;
    case LWFTP_CLOSING:
      // this function frees s, no use of s is allowed after
      lwftp_control_close(s, result);
    default:;
  }
}

/** Start a RETR data session
 * @param pointer to lwftp session
 */
static void lwftp_start_RETR(void *arg)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if ( s->control_state == LWFTP_LOGGED ) {
    lwftp_send_msg(s, PTRNLEN("TYPE I\r\n"));
    s->control_state = LWFTP_TYPE_SENT;
    s->target_state = LWFTP_RETR_SENT;
  } else {
    tPrintf("Unexpected condition");
    if (s->done_fn) s->done_fn(s->handle, LWFTP_RESULT_ERR_INTERNAL);
  }
}

/** Start a MLSD data session
 * @param pointer to lwftp session
 */
static void lwftp_start_MLSD(void *arg)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if ( s->control_state == LWFTP_LOGGED ) {
    lwftp_send_msg(s, PTRNLEN("TYPE I\r\n"));
    s->control_state = LWFTP_TYPE_SENT;
    s->target_state = LWFTP_MLSD_SENT;
  } else {
    tPrintf("Unexpected condition");
    if (s->done_fn) s->done_fn(s->handle, LWFTP_RESULT_ERR_INTERNAL);
  }
}

/** Start a STOR data session
 * @param pointer to lwftp session
 */
static void lwftp_start_STOR(void *arg)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if ( s->control_state == LWFTP_LOGGED ) {
    lwftp_send_msg(s, PTRNLEN("TYPE I\n"));
    s->control_state = LWFTP_TYPE_SENT;
    s->target_state = LWFTP_STOR_SENT;
  } else {
    tPrintf("Unexpected condition");
    if (s->done_fn) s->done_fn(s->handle, LWFTP_RESULT_ERR_INTERNAL);
  }
}

/** Send QUIT to terminate control session
 * @param pointer to lwftp session
 */
static void lwftp_send_QUIT(void *arg)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if (s->control_pcb) {
    lwftp_send_msg(s, PTRNLEN("QUIT\n"));
    tcp_output(s->control_pcb);
    s->control_state = LWFTP_QUIT_SENT;
  }
}

/** Handle control connection incoming data
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param pointer to incoming pbuf
 * @param state of incoming process
 */
static err_t lwftp_control_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if ( err == ERR_OK ) {
    if (p) {
      tcp_recved(tpcb, p->tot_len);
      lwftp_control_process(s, tpcb, p);
    } else {
      tPrintf("lwftp:connection closed by remote host\n");
      lwftp_control_close(s, LWFTP_RESULT_ERR_CLOSED);
    }
  } else {
    tPrintf("lwftp:failed to receive (%s)\n",lwip_strerr(err));
    lwftp_control_close(s, LWFTP_RESULT_ERR_UNKNOWN);
  }
  return err;
}

/** Handle control connection acknowledge of sent data
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param number of bytes sent
 */
static err_t lwftp_control_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  tPrintf("lwftp:successfully sent %d bytes\n",len);
  return ERR_OK;
}

/** Handle control connection error
 * @param pointer to lwftp session data
 * @param state of connection
 */
static void lwftp_control_err(void *arg, err_t err)
{
  LWIP_UNUSED_ARG(err);
  if (arg != NULL) {
    lwftp_session_t *s = (lwftp_session_t*)arg;
    int result;
    if( s->control_state == LWFTP_CLOSED ) {
      tPrintf("lwftp:failed to connect to server (%s)\n",lwip_strerr(err));
      result = LWFTP_RESULT_ERR_CONNECT;
    } else {
      tPrintf("lwftp:connection closed by remote host\n");
      result = LWFTP_RESULT_ERR_CLOSED;
    }
    s->control_pcb = NULL; // No need to de-allocate PCB
    lwftp_control_close(s, result);
  }
}


/** Process newly connected PCB
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param state of connection
 */
static err_t lwftp_control_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  lwftp_session_t *s = (lwftp_session_t*)arg;

  if ( err == ERR_OK ) {
    tPrintf("lwftp:connected to server\n");
      s->control_state = LWFTP_CONNECTED;
  } else {
    tPrintf("lwftp:err in control_connected (%s)\n",lwip_strerr(err));
  }
  return err;
}


/** Open a control session
 * @param Session structure
 */
err_t lwftp_connect(lwftp_session_t *s)
{
  err_t error;
  enum lwftp_results retval = LWFTP_RESULT_ERR_UNKNOWN;

  // Check user supplied data
  if ( (s->control_state != LWFTP_CLOSED) ||
       s->control_pcb ||
       s->data_pcb ||
       !s->user ||
       !s->pass )
  {
    tPrintf("lwftp:invalid control session\n");
    retval = LWFTP_RESULT_ERR_ARGUMENT;
    goto exit;
  }
  // Get sessions pcb
  s->control_pcb = tcp_new();
  if (!s->control_pcb) {
    tPrintf("lwftp:cannot alloc control_pcb (low memory?)\n");
    retval = LWFTP_RESULT_ERR_MEMORY;
    goto exit;
  }
  // Open control session
  tcp_arg(s->control_pcb, s);
  tcp_err(s->control_pcb, lwftp_control_err);
  tcp_recv(s->control_pcb, lwftp_control_recv);
  tcp_sent(s->control_pcb, lwftp_control_sent);
  error = tcp_connect(s->control_pcb, &s->server_ip, s->server_port, lwftp_control_connected);
  if ( error == ERR_OK ) {
    retval = LWFTP_RESULT_INPROGRESS;
    goto exit;
  }

  // Release pcbs in case of failure
  tPrintf("lwftp:cannot connect control_pcb (%s)\n", lwip_strerr(error));
  lwftp_control_close(s, -1);

exit:
  if (s->done_fn) s->done_fn(s->handle, retval);
  return retval;
}


/** Retrieve data from a remote file
 * @param Session structure
 */
err_t lwftp_retrieve(lwftp_session_t *s)
{
  err_t error;
  enum lwftp_results retval = LWFTP_RESULT_ERR_UNKNOWN;

  // Check user supplied data
  if ( (s->control_state!=LWFTP_LOGGED) ||
       !s->remote_path ||
       s->data_pcb )
  {
    tPrintf("lwftp:invalid session data\n");
    retval = LWFTP_RESULT_ERR_ARGUMENT;
    goto exit;
  }
  // Get data pcb
  s->data_pcb = tcp_new();
  if (!s->data_pcb) {
    tPrintf("lwftp:cannot alloc data_pcb (low memory?)\n");
    retval = LWFTP_RESULT_ERR_MEMORY;
    goto exit;
  }
  // Initiate transfer
  error = tcpip_callback(lwftp_start_RETR, s);
  if ( error == ERR_OK ) {
    retval = LWFTP_RESULT_INPROGRESS;
  } else {
    tPrintf("cannot start RETR (%s)",lwip_strerr(error));
    retval = LWFTP_RESULT_ERR_INTERNAL;
  }

exit:
  if (s->done_fn) s->done_fn(s->handle, retval);
  return retval;
}

/** Retrieve data from a remote file
 * @param Session structure
 */
err_t lwftp_mlsd(lwftp_session_t *s)
{
  err_t error;
  enum lwftp_results retval = LWFTP_RESULT_ERR_UNKNOWN;

  // Check user supplied data
  if ( (s->control_state!=LWFTP_LOGGED) ||
       s->data_pcb )
  {
    tPrintf("lwftp:invalid session data\n");
    retval = LWFTP_RESULT_ERR_ARGUMENT;
    goto exit;
  }
  // Get data pcb
  s->data_pcb = tcp_new();
  if (!s->data_pcb) {
    tPrintf("lwftp:cannot alloc data_pcb (low memory?)\n");
    retval = LWFTP_RESULT_ERR_MEMORY;
    goto exit;
  }
  // Initiate transfer
  error = tcpip_callback(lwftp_start_MLSD, s);
  if ( error == ERR_OK ) {
    retval = LWFTP_RESULT_INPROGRESS;
  } else {
    tPrintf("cannot start MLSD (%s)",lwip_strerr(error));
    retval = LWFTP_RESULT_ERR_INTERNAL;
  }

exit:
  if (s->done_fn) s->done_fn(s->handle, retval);
  return retval;
}

/** Store data to a remote file
 * @param Session structure
 */
err_t lwftp_store(lwftp_session_t *s)
{
  err_t error;
  enum lwftp_results retval = LWFTP_RESULT_ERR_UNKNOWN;

  // Check user supplied data
  if ( (s->control_state!=LWFTP_LOGGED) ||
       !s->remote_path ||
       s->data_pcb )
  {
    tPrintf("lwftp:invalid session data\n");
    retval = LWFTP_RESULT_ERR_ARGUMENT;
    goto exit;
  }
  // Get data pcb
  s->data_pcb = tcp_new();
  if (!s->data_pcb) {
    tPrintf("lwftp:cannot alloc data_pcb (low memory?)\n");
    retval = LWFTP_RESULT_ERR_MEMORY;
    goto exit;
  }
  // Initiate transfer
  error = tcpip_callback(lwftp_start_STOR, s);
  if ( error == ERR_OK ) {
    retval = LWFTP_RESULT_INPROGRESS;
  } else {
    tPrintf("cannot start STOR (%s)",lwip_strerr(error));
    retval = LWFTP_RESULT_ERR_INTERNAL;
  }

exit:
  if (s->done_fn) s->done_fn(s->handle, retval);
  return retval;
}


/** Terminate FTP session
 * @param Session structure
 */
void lwftp_close(lwftp_session_t *s)
{
  err_t error;

  // Nothing to do when already closed
  if ( s->control_state == LWFTP_CLOSED ) return;

  // Initiate transfer
  error = tcpip_callback(lwftp_send_QUIT, s);
  if ( error != ERR_OK ) {
    // This is a critical error, try to close anyway
    // polling process may save us
    tPrintf("cannot request for close");
    s->control_state = LWFTP_QUIT;
  }
}

__weak void ftp_retr_callback(void *arg, int result)
{
   lwftp_session_t *s;

   s = (lwftp_session_t*)arg;
   if ( result != LWFTP_RESULT_OK ) 
   {
      tPrintf("retr failed (%d)", result);
      lwftp_close(s);
      return;
    }
    // Test is done
    lwftp_close(s);
}

__weak uint32_t data_sink(void *arg, const char* ptr, uint32_t len)
{
    static const uint32_t mylen = 12345;
    static char * const myconfig = (char*)0x20000000;
    static uint32_t offset = 0;

    if (ptr) 
    {
      if (len > mylen-offset)
      {
        len = mylen-offset;
      }
        
      memcpy( myconfig+offset, ptr, len );
      offset += len;
    }
    return len;
}

static uint32_t data_list(void *arg, const char* ptr, uint32_t len)
{
  char* pFileSize;


  if (ptr) 
  {
    tPrintf("%s\r\n", ptr);
    pFileSize = strstr(ptr, fileToRetr);
    if (pFileSize != NULL) 
    {
      /* now find the file string size; the string is  xxxx Filename where xxxx is the length in decimal  */
      pFileSize--;
      *pFileSize = '\0'; 
      while (*pFileSize != ' ') 
      {
        pFileSize--;
      }
      pFileSize++; // now pointing to decimal size
      fileSize = atoi(pFileSize);
      tPrintf("File %s has size = %d\r\n", fileToRetr, fileSize);
    }
  }
  return fileSize;
}

static void ftp_mlsd_callback(void *arg, int result)
{
   lwftp_session_t *s;

   s = (lwftp_session_t*)arg;
   if ( result == LWFTP_RESULT_INPROGRESS ) 
   {
      return;
   }
   if ( result != LWFTP_RESULT_OK ) 
   {
      tPrintf("msld failed (%d)", result);
      lwftp_close(s);
      return;
    }
    // Test is done
    lwftp_close(s);
}


__weak void ftp_stor_callback(void *arg, int result)
{
    lwftp_session_t *s;  
    err_t error;

    s = (lwftp_session_t*)arg;
    if ( result != LWFTP_RESULT_OK ) 
    {
        tPrintf("stor failed (%d)", result);
        lwftp_close(s);
        return;
    }
#ifdef CONTINUE_RETR
    // Continue with RETR request
    s->data_sink = data_sink;
    s->done_fn = ftp_retr_callback;
    s->remote_path = "configfile";
    error = lwftp_retrieve(s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        tPrintf("lwftp_retrieve failed (%d)", error);
    }
    // FTP session will continue with RETR and sink callbacks
#else
    // Continue with MLSD request
    s->data_sink = data_list;
    s->done_fn = ftp_mlsd_callback;
    s->remote_path = "";
    tPrintf("ftp_stor_callback\r\n");
    error = lwftp_retrieve(s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        tPrintf("lwftp_retrieve failed (%d)", error);
    }
    // FTP session will continue with RETR and sink callbacks
#endif
}

#ifdef CMD_STORE
static uint data_source(void *arg, const char** pptr, uint maxlen)
{
    static const uint mylen = 12345;
    static const char * const mydata = (char*)0x20000000;
    static uint offset = 0;
    uint len = 0;

    // Check for data request or data sent notice
    if (pptr) {
        len = mylen - offset;
        if ( len > maxlen ) len = maxlen;
        *pptr = mydata + offset;
    } else {
        offset += maxlen;
        if ( offset > mylen ) offset = mylen;
    }
    return len;
}


static void ftp_connect_callback(void *arg, int result)
{
    lwftp_session_t *s = (lwftp_session_t)arg;
    err_t error;

    if ( result != LWFTP_RESULT_LOGGED ) {
        tPrintf("login failed (%d)", result);
        return lwftp_close(s);
    }
    // Continue with STOR request
    s->data_source = data_source;
    s->done_fn = ftp_stor_callback;
    s->remotepath = "logfile";
    error = lwftp_store(s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        tPrintf("lwftp_store failed (%d)", error);
    }
    // FTP session will continue with STOR and source callbacks
}
#endif

static void ftp_connect_callback(void *arg, int result)
{
    lwftp_session_t *s;
    err_t error;

    s = (lwftp_session_t*)arg;
    if ( result != LWFTP_RESULT_LOGGED ) 
    {
        tPrintf("login failed (%d)", result);
        lwftp_close(s);
        return;
    }
#ifdef CONTINUE_RETR
    // Continue with RETR request
    s->data_sink = data_sink;
    s->done_fn = ftp_retr_callback;
    s->remote_path = "LastSerial.txt";
    error = lwftp_retrieve(s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        tPrintf("lwftp_retrieve failed (%d)", error);
    }
    // FTP session will continue with RETR and sink callbacks
#else
    // Continue with MLSD request
    tPrintf("ftp_connect_callback\r\n");
    s->data_sink = data_list;
    s->done_fn = ftp_mlsd_callback;
    s->remote_path = "";
    error = lwftp_mlsd(s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        tPrintf("lwftp_retrieve failed (%d)", error);
    }
    // FTP session will continue with MLSD and sink callbacks
#endif
}


void ftp_test(void)
{
    static lwftp_session_t s;   // static content for the whole FTP session
    err_t error;

    // Initialize session data
    memset(&s, 0, sizeof(s));
    IP4_ADDR(&s.server_ip, 192,168,1,110);
    s.server_port = 21;
    s.done_fn = ftp_connect_callback;
    s.user = "Nicolino";
    s.pass = "nicolino63";
    // We have no extra user data, simply use the session structure
    s.handle = &s;

    // Start the connection state machine
    error = lwftp_connect(&s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        tPrintf("lwftp_connect failed (%d)", error);
    }
    // FTP session will continue with the connection callback
}
#endif


