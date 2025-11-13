#define  GIGADEVICE
#define  ECC
//#define  DEBUG_TEST

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;   // Per File.ReadAllBytes (per leggere il file binario reale)
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Security.Cryptography.X509Certificates;
using System.Text; // Per Encoding.UTF8.GetBytes (per l'esempio)


public class FirmaECC
{
    // Genera una coppia di chiavi ECC (nistP256)
    public static ECDsa CreaChiavePrivata()
    {
        return ECDsa.Create(ECCurve.NamedCurves.nistP256);
    }

    // Firma un messaggio con SHA-256
    public static byte[] FirmaMessaggio(string messaggio, ECDsa chiavePrivata)
    {
        byte[] dati = Encoding.UTF8.GetBytes(messaggio);
        return chiavePrivata.SignData(dati, HashAlgorithmName.SHA256);
    }

    // Verifica la firma
    public static bool VerificaFirma(string messaggio, byte[] firma, ECDsa chiavePubblica)
    {
        byte[] dati = Encoding.UTF8.GetBytes(messaggio);
        return chiavePubblica.VerifyData(dati, firma, HashAlgorithmName.SHA256);
    }
}

public class RsaSignatureGenerator
{
    /// <summary>
    /// Calcola l'hash SHA-256 di un byte array.
    /// </summary>
    /// <param name="data">Il byte array da cui calcolare l'hash.</param>
    /// <returns>L'hash SHA-256 come byte array.</returns>
    public static byte[] CalculateSha256Hash(byte[] data)
    {
        if (data == null)
        {
            throw new ArgumentNullException(nameof(data), "I dati da hashare non possono essere nulli.");
        }

        using (SHA256 sha256 = SHA256.Create())
        {
            return sha256.ComputeHash(data);
        }
    }

    /// <summary>
    /// Genera una firma digitale utilizzando un hash, una chiave privata RSA (in formato PEM)
    /// e il padding PKCS#1.
    /// </summary>
    /// <param name="hashToSign">L'hash pre-calcolato (es. SHA-256) del dato da firmare.</param>
    /// <param name="privateKeyPem">La chiave privata RSA in formato PEM (stringa).</param>
    /// <returns>La firma digitale come byte array.</returns>
    /// <exception cref="ArgumentNullException">Se l'hash o la chiave PEM sono nulli.</exception>
    /// <exception cref="CryptographicException">In caso di errori durante il caricamento della chiave o la firma.</exception>
    public static byte[] GenerateRsaPkcs1Signature(byte[] hashToSign, string privateKeyPem)
    {
        if (hashToSign == null)
        {
            throw new ArgumentNullException(nameof(hashToSign), "L'hash da firmare non può essere nullo.");
        }
        if (string.IsNullOrWhiteSpace(privateKeyPem))
        {
            throw new ArgumentException("La chiave privata PEM non può essere nulla o vuota.", nameof(privateKeyPem));
        }

        using (RSA rsa = RSA.Create())
        {
            try
            {
                // Importa la chiave privata dal formato PEM.
                // Questo metodo supporta PEM con header come "-----BEGIN RSA PRIVATE KEY-----" o "-----BEGIN PRIVATE KEY-----".
                rsa.ImportFromPem(privateKeyPem);

                // Genera la firma digitale.
                // Il primo parametro è l'hash che vogliamo firmare.
                // Il secondo parametro indica l'algoritmo usato per generare quell'hash (SHA256).
                // Il terzo parametro specifica il tipo di padding (PKCS#1 v1.5 in questo caso).
                return rsa.SignHash(hashToSign, HashAlgorithmName.SHA256, RSASignaturePadding.Pkcs1);
            }
            catch (CryptographicException ex)
            {
                throw new CryptographicException("Errore durante la generazione della firma RSA. Controllare il formato o la validità della chiave privata PEM.", ex);
            }
            catch (Exception ex)
            {
                // Cattura altri errori inaspettati
                throw new InvalidOperationException("Si è verificato un errore inatteso durante l'operazione di firma.", ex);
            }
        }
    }
    public static void GenerateModAndExp(string publicKeyPem, out string mod, out string exp)
    {
        if (string.IsNullOrWhiteSpace(publicKeyPem))
        {
            throw new ArgumentException("La chiave publica PEM non può essere nulla o vuota.", nameof(publicKeyPem));
        }

        using (RSA rsa = RSA.Create())
        {
            try
            {
                // Importa la chiave pubblica dal formato PEM.
                // Questo metodo supporta PEM con header come "-----BEGIN RSA PUBLIC KEY-----" o "-----BEGIN PUBLIC KEY-----".
                rsa.ImportFromPem(publicKeyPem);

                // Ottieni il modulo e l'esponente
                RSAParameters publicKeyRead = rsa.ExportParameters(false); // false indica solo la chiave pubblica


                // convertiamo la chiave pubblica nel formato modulo esponente usato nelle libreria di ST
                mod = BitConverter.ToString(publicKeyRead.Modulus).Replace("-", "");
                exp = BitConverter.ToString(publicKeyRead.Exponent).Replace("-", "");

            }
            catch (CryptographicException ex)
            {
                throw new CryptographicException("Errore durante la generazione della firma RSA. Controllare il formato o la validità della chiave privata PEM.", ex);
            }
            catch (Exception ex)
            {
                // Cattura altri errori inaspettati
                throw new InvalidOperationException("Si è verificato un errore inatteso durante l'operazione di firma.", ex);
            }
        }
    }
    public static byte[] StringToByteArray(string hex)
    {
        return Enumerable.Range(0, hex.Length).Where(x => x % 2 == 0).Select(x => Convert.ToByte(hex.Substring(x, 2), 16)).ToArray();
    }

    public static void Main(string[] args)
    {
        string line;
        string[] strArr = null;
        string[] strArrDir = null;
        char[] splitchar = { '<', '>' };

        string strBinBootFileName = "";
        string strBinFwFileName = "";
        string strBinOutAllFileName = "";
        string strHexOutAllFileName = "";
        string strBinOutFwFileName = "";
        string outFile = "";
        string outDir = "";
        string outNoBoot = "";
        string nameOutFile = "";
        bool nameNoBoot = false;
        bool dirFound = false;
        int lenNorm = 2048;

#if DEBUG_TEST
        string currentDir = "C:\\project\\scame\\SCU_GD_RED\\SCAME_CONTROL_UNIT\\trunk\\Win";
#else
        string currentDir = System.IO.Directory.GetCurrentDirectory();
#endif
        Console.WriteLine("Current dir = " + currentDir);
        int pos = currentDir.IndexOf("Win");
        if (pos < 0)
        {
            pos = currentDir.IndexOf("EWARM");
        }
        string pathFile = currentDir.Remove(pos);

        pos = currentDir.IndexOf("SCAME_CONTROL_UNIT");
        if (pos < 0)
        {
            pos = currentDir.IndexOf("EWARM");
        }
        string radixPath = currentDir.Remove(pos);

        string bootPath = radixPath + "SCAME_BOOT\\trunk\\EWARM\\BootSCU\\Exe";
        string fwPath = pathFile + "EWARM\\TARGET\\Exe";
        string srecPath = pathFile + "EWARM\\srec_cat.exe ";
        //string keyPubFileName = pathFile + "EWARM\\pubkeyEcc.pem ";
        string keyPubFileName = pathFile + "EWARM\\pubkeyEccRawDbg.bin ";
        //string keyPrivFileName = pathFile + "EWARM\\keyEcc.pem ";
        string keyPrivFileName = pathFile + "EWARM\\keyEccRawDbg.bin";
        string signatureFileName = pathFile + "EWARM\\signature.bin";

        Console.WriteLine("Path = " + pathFile);
        Console.WriteLine("Path = " + radixPath);
        Console.WriteLine("Path = " + bootPath);
        Console.WriteLine("Path = " + fwPath);
        Console.WriteLine("CheckTag Version V5.0");
        //.Replace("\\bin\\Debug", "") + "\\Data\\LogResult";
        //string strBinFileName = "..\\..\\..\\..\\MDK-ARM\\DSPMV1X16SAUC\\DSPMV1X16SAUC_BOOT.bin";


#if GIGADEVICE
        var allFiles = Directory.GetFiles(bootPath, "BOOTGD_V5*.bin", SearchOption.TopDirectoryOnly);
            if ((allFiles.Length == 1) && (allFiles[0].Contains("BOOTGD") == true))
#else
        var allFiles = Directory.GetFiles(bootPath, "BOOT_V1*.bin", SearchOption.TopDirectoryOnly);
        if ((allFiles.Length == 1) && (allFiles[0].Contains("BOOT_") == true))
#endif
        {
            strBinBootFileName = allFiles[0];
#if GIGADEVICE
                strBinFwFileName = fwPath + "\\SCU_GDF4xx.bin"; 
#else
            strBinFwFileName = fwPath + "\\ScameSCU.bin";
#endif
            strBinOutAllFileName = fwPath + "\\ScameDbgSCUBoot_";
            strBinOutFwFileName = fwPath + "\\ScameDbgSCU_";
            int cnt;
            System.Threading.Thread.Sleep(1000);
            if ((strBinBootFileName != "") && (strBinFwFileName != ""))
            {
                if ((File.Exists(strBinBootFileName)) && (File.Exists(strBinFwFileName)))
                {
                    int bootSectorSize = (int)0x8000;
                    int secureAreaSize = (int)0x4000;
                    int signatureAreaSize = lenNorm;
                    int secureAreaStart = (int)0x8000;
                    int pubKeyAddress = (int)0x7000;
                    int infoBoot = (int)0x1000;
                    int infoBootStr = (int)0x1005;
                    int infoBootCks = (int)0x100A;
                    int infoBootSize = (int)0x10;
                    int checkSumOffset = (int)0x1000;     /* 0x9000 dall'origine */
                    int lenOffset = checkSumOffset + 40;  /* infoNewFw_st {u32 fwchecksum; u8 strCheck[16]; u8 fwVersion[16]; u32 offsetAddr; u32 fwLen} */
                    string modulo;
                    String esponente;

                    Byte[] bytesBoot = File.ReadAllBytes(strBinBootFileName);
                    Byte[] bytesFw = File.ReadAllBytes(strBinFwFileName);
                    Byte[] keyPubBytes = File.ReadAllBytes(keyPubFileName);
                    // --------- CARICA LA CHIAVE PRIVATA ----------
#if false
                    string rsaPrivateKeyPem = File.ReadAllText(keyPrivFileName);
                    byte[] derPriv = Convert.FromBase64String(
                        rsaPrivateKeyPem.Replace("-----BEGIN PRIVATE KEY-----", "")
                               .Replace("-----END PRIVATE KEY-----", "")
                               .Replace("\n", ""));
                    ECDsa chiavePrivata = ECDsa.Create();
                    chiavePrivata.ImportECPrivateKey(derPriv, out _);
                    ///chiavePrivata.ImportPkcs8PrivateKey(derPriv, out _);
                    string rsaPublicKeyPem = File.ReadAllText(keyPubFileName);
                    byte[] derPub = Convert.FromBase64String(
                        rsaPublicKeyPem.Replace("-----BEGIN PRIVATE KEY-----", "")
                               .Replace("-----END PRIVATE KEY-----", "")
                               .Replace("\n", ""));
#else
                    byte[] rawPrivateKey = File.ReadAllBytes(keyPrivFileName);
                    byte[] rawPublicKey  = File.ReadAllBytes(keyPubFileName);

                    // 📦 Importa parametri ECC
                    ECParameters ecParams = new ECParameters
                    {
                        Curve = ECCurve.NamedCurves.nistP256,
                        D = rawPrivateKey,
                        Q = new ECPoint
                        {
                            X = rawPublicKey[..32],
                            Y = rawPublicKey[32..]
                        }
                    };

                    // 🔧 Crea ECDsa con chiave privata
                    using ECDsa ecdsa = ECDsa.Create();
                    ecdsa.ImportParameters(ecParams);

#endif


                    int bootLen = bytesBoot.Length;
                    int fwLen = bytesFw.Length;
                    int fwLenNorm = fwLen;
                    if ((fwLenNorm % (int)lenNorm) != 0)
                    {
                        fwLenNorm = ((fwLen / (int)lenNorm) + 1) * (int)lenNorm;
                    }
                    int totalLen = (bootSectorSize + secureAreaSize) + fwLenNorm + lenNorm;

                    /* checksum for boot file */
                    Byte[] bootBuff = new Byte[bootSectorSize + secureAreaSize];
                    for (cnt = 0; cnt < bootSectorSize + secureAreaSize; cnt++)
                    {
                        bootBuff[cnt] = (Byte)0xFF;
                    }
                    for (cnt = 0; cnt < bootLen; cnt++)
                    {
                        bootBuff[cnt] = bytesBoot[cnt];
                    }

#if true
                    /* la memorizzazione della chiave pubblica è stata riassegnata al posst build */
                    /* aggiungiamo la chiave pubblica al file del boot */
                    for (cnt = 0; cnt < rawPublicKey.Length; cnt++)
                    {
                        bootBuff[pubKeyAddress + cnt] = rawPublicKey[cnt];
                    }
#endif
                    int cksB = 0;
                    for (cnt = 0; cnt < bootSectorSize; cnt++)
                    {
                        if ((cnt < infoBoot) || (cnt >= (infoBoot + infoBootSize)))
                        {
                            cksB += (int)bootBuff[cnt];
                        }
                        if ((cnt != 0) && ((cnt % (int)0x1000) == 0))
                        {
                            cksB &= (int)0x00FFFFFF;
                        }
                    }
                    /* nella uint32_t riservata per la checksum scriviamo  questa checksum appena trovata */
                    int ixB = infoBootCks;
                    byte valB = (byte)(cksB);
                    bootBuff[ixB] = valB; ixB++;
                    valB = (byte)(cksB >> 8);
                    bootBuff[ixB] = valB; ixB++;
                    valB = (byte)(cksB >> 16);
                    bootBuff[ixB] = valB; ixB++;
                    valB = (byte)(cksB >> 24);
                    bootBuff[ixB] = valB;
                    /* sostituisco '\0' della versione FW con spazio bianco Questo permette a strstr che */
                    /* cercherà la stringa DSCU di funzionare correttamente                              */ 
                    bootBuff[infoBootStr - 1] = (byte)' ';
                    /* nell'array riservato per la chiave scriviamo "BSCU" */
                    bootBuff[infoBootStr] = (byte)'B'; bootBuff[infoBootStr + 1] = (byte)'S'; bootBuff[infoBootStr + 2] = (byte)'C'; bootBuff[infoBootStr + 3] = (byte)'U';

                    Byte[] bBuff = new Byte[totalLen];      // bBuff contiene tutto boot+securArea+FwApplicativo+Signature
                    Byte[] bBuff_Fw = new Byte[fwLenNorm];  // bBuff_Fw contiene solo il firmware applicativo
                    for (cnt = 0; cnt < totalLen; cnt++)
                    {
                        bBuff[cnt] = (Byte)0xFF;
                    }
                    for (cnt = 0; cnt < fwLenNorm; cnt++)
                    {
                        bBuff_Fw[cnt] = (Byte)0xFF;
                    }

                    bootBuff.CopyTo(bBuff, 0);              /* copio nel buffer temporaneo il codice del boot */
                    bytesFw.CopyTo(bBuff_Fw, 0);            /* copio nel buffer temporaneo multiplo lenNorm (era 1024, ora 2048)  il codice del FW */
                    /* memorizzo la lunghezza del file su cui calcolare la checksum */
                    int ix = lenOffset;              // 0x1000 + 40 = 0x1028
                    byte val = (byte)(fwLenNorm);
                    bBuff_Fw[ix] = val; ix++;
                    val = (byte)(fwLenNorm >> 8);
                    bBuff_Fw[ix] = val; ix++;
                    val = (byte)(fwLenNorm >> 16);
                    bBuff_Fw[ix] = val; ix++;
                    val = (byte)(fwLenNorm >> 24);
                    bBuff_Fw[ix] = val;
                    /* calcolo della checksum su 32bit del codice del FW */
                    UInt32 cks = 0;
                    for (cnt = 0; cnt < fwLenNorm; cnt++) 
                    {
                        cks += (UInt32)bBuff_Fw[cnt];  /* assicurarsi che il firmware in fwInfo.fwchecksum abbia messo 0 */
                    }
                    /* nella uint32_t riservata per la checksum scriviamo  questa checksum appena trovata */
                    ix = checkSumOffset;
                    val = (byte)(cks);
                    bBuff_Fw[ix] = val; ix++;
                    val = (byte)(cks >> 8);
                    bBuff_Fw[ix] = val; ix++;
                    val = (byte)(cks >> 16);
                    bBuff_Fw[ix] = val; ix++;
                    val = (byte)(cks >> 24);
                    bBuff_Fw[ix] = val;


                    /* aggiungo al file del boot il file del FW signature compresa */
                    bBuff_Fw.CopyTo(bBuff, bootSectorSize + secureAreaSize);  /* copio nel buffer temporaneo il codice del FW dopo l'area riservata al boot  */

                    // creo il file boot+SecurArea+FwApplicativo
                    Byte[] bBuff_For_Signature = new Byte[bootSectorSize + secureAreaSize + fwLenNorm];
                    for (cnt = 0; cnt < bootSectorSize + secureAreaSize + fwLenNorm; cnt++)
                    {
                        bBuff_For_Signature[cnt] = bBuff[cnt];
                    }
                    // Calcola l'hash SHA-256 del contenuto binario di tutto il file boot+SecurArea+FwApplicativo
                    byte[] hash = CalculateSha256Hash(bBuff_For_Signature);
                    Console.WriteLine($"Hash SHA-256 calcolato (Base64): {Convert.ToBase64String(hash)}");
#if ECC
                    byte[] signature = ecdsa.SignData(bBuff_For_Signature, HashAlgorithmName.SHA256);
#else
                    // Genera la firma digitale con la chiave privata
                    byte[] signature = GenerateRsaPkcs1Signature(hash, rsaPrivateKeyPem);
#endif
                    Console.WriteLine($"\nFirma digitale generata (Base64): {Convert.ToBase64String(signature)}");
                    /* copio la firma nell'area ad essa riservata ovvero ultimi 2048 bytes */
                    int j;
                    for (cnt = bootSectorSize + secureAreaSize + fwLenNorm, j = 0; j < signature.Length; cnt++, j++)
                    {
                        bBuff[cnt] = signature[j]; /* fine file, ultimi 2K appositamente dedicati */
                    }


                    /* adesso passiamo alla generazione dei file <solo fw>.bin <boot+fw>.bin e <boot+fw>.hex */
                    ix = 0x1000 + 20;
                    int idx = 0;
                    while ((bytesFw[ix] != '\0') && (idx < 20))
                    {
                        if (bytesFw[ix] != '.')
                        {
                            idx++;
                        }
                        ix++;
                    }

                    char[] byteVer = new char[idx];
                    ix = 0x1000 + 20;
                    idx = 0;
                    while ((bBuff_Fw[ix] != '\0') && (idx < 20))
                    {
                        if (bBuff_Fw[ix] != '.')
                        {
                            byteVer[idx] = (char)bBuff_Fw[ix];
                            idx++;
                        }
                        ix++;
                    }

                    string charsVer = new string(byteVer);
                    strHexOutAllFileName = strBinOutAllFileName + charsVer + ".hex";
                    strBinOutAllFileName += (charsVer + ".bin");
                    strBinOutFwFileName += (charsVer + ".bin");
                    File.WriteAllBytes(strBinOutAllFileName, bBuff);
                    /* nella versione RED questo file è inutile: usiamo solo quello con boot 
                    File.WriteAllBytes(strBinOutFwFileName, bBuff_Fw);  genero file di lunghezza normalizzata allineata a lenNorm */

                    Process ExternalProcess = new Process();
                    //ExternalProcess.StartInfo.FileName = fwPath + "\\intelOut.bat";
                    ExternalProcess.StartInfo.FileName = "cmd.exe";
                    //ExternalProcess.StartInfo.Arguments = strBinOutAllFileName;
                    //ExternalProcess.StartInfo.Arguments = @"/K " + fwPath + "\\intelOut.bat ScameSCUBoot_V234.bin pippotto.hex";
                    ExternalProcess.StartInfo.Arguments = @"/K " + srecPath + strBinOutAllFileName + " -Binary -offset 0x8000000 -o " + strHexOutAllFileName + " -Intel";
                    //ExternalProcess.StartInfo.Arguments = "ScameSCUBoot_V234.bin -Binary -offset 0x8000000 -o ScameSCUBoot_V235.hex -Intel";
                    ExternalProcess.StartInfo.CreateNoWindow = false;
                    ExternalProcess.StartInfo.UseShellExecute = true;
                    ExternalProcess.StartInfo.WindowStyle = ProcessWindowStyle.Hidden;
                    //ExternalProcess.StartInfo.WindowStyle = ProcessWindowStyle.Maximized;
                    ExternalProcess.Start();
                    //ExternalProcess.WaitForExit();


                }
                else
                {
                    Console.WriteLine("Bin File name doesn't exist");
                }
                // file exist
            } // file name empty
        }
        else
        {
            Console.WriteLine("Bin Boot file not found");
        }

    }
}

#if false


            // --- PARTE 1: SIMULAZIONE DEL FILE BINARIO E CALCOLO DELL'HASH ---
            // In un'applicazione reale, leggeresti il contenuto del tuo file binario così:
            // byte[] fileContent = File.ReadAllBytes("C:\\Percorso\\AlTuo\\FileBinario.bin");

            // Per l'esempio, creiamo un byte array di prova.
            byte[] binaryFileData = Encoding.UTF8.GetBytes("Questo è un contenuto di esempio per il file binario.");
            Console.WriteLine($"Dimensione dei dati binari: {binaryFileData.Length} bytes");

            // Calcola l'hash SHA-256 del contenuto binario
            byte[] hash = CalculateSha256Hash(binaryFileData);
            Console.WriteLine($"Hash SHA-256 calcolato (Base64): {Convert.ToBase64String(hash)}");

            // --- PARTE 2: CARICAMENTO DELLA CHIAVE PRIVATA PEM E GENERAZIONE DELLA FIRMA ---

            // ATTENZIONE: La tua chiave privata RSA in formato PEM.
            // NON INSERIRE MAI CHIAVI PRIVATE SENSIBILI DIRETTAMENTE NEL CODICE DI PRODUZIONE!
            // Dovrebbero essere caricate in modo sicuro (es. da un Key Vault, un file con permessi ristretti,
            // o un'altra soluzione di gestione delle chiavi sicura).
            string rsaPublicKeyPem = File.ReadAllText("C:\\temp\\testCsharp\\ConsoleApp1\\ConsoleApp1\\pubkey.pem");
            string rsaPrivateKeyPem = File.ReadAllText("C:\\temp\\testCsharp\\ConsoleApp1\\ConsoleApp1\\key.pem");

            // NOTA: Se non hai una chiave privata RSA e vuoi generarne una per test:
            // using (RSA rsaTest = RSA.Create(2048)) // Crea una chiave RSA a 2048 bit
            // {
            //     Console.WriteLine("\n--- Chiave di test generata ---");
            //     Console.WriteLine("Chiave Pubblica (PEM):\n" + rsaTest.ExportRSAPublicKeyPem());
            //     Console.WriteLine("Chiave Privata (PEM):\n" + rsaTest.ExportRSAPrivateKeyPem());
            //     Console.WriteLine("------------------------------");
            //     // Copia la chiave privata generata da qui nel rsaPrivateKeyPem sopra per i tuoi test.
            // }


            try
            {
                // Genera la firma digitale con la chiave privata
                byte[] signature = GenerateRsaPkcs1Signature(hash, rsaPrivateKeyPem);
                Console.WriteLine($"\nFirma digitale generata (Base64): {Convert.ToBase64String(signature)}");

                // --- PARTE 3 (OPZIONALE): VERIFICA DELLA FIRMA (per dimostrazione) ---
                // Per verificare la firma, avresti bisogno della chiave pubblica corrispondente.
                // In un'applicazione reale, la verifica verrebbe eseguita da un'entità che ha solo la chiave pubblica.

                /*
                string rsaPublicKeyPem = @"-----BEGIN PUBLIC KEY-----
                MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAsYw1z... (la tua chiave pubblica RSA) ...
                -----END PUBLIC KEY-----";

                using (RSA rsaPublic = RSA.Create())
                {
                    rsaPublic.ImportFromPem(rsaPublicKeyPem);
                    bool isValid = rsaPublic.VerifyHash(
                        hash,         // L'hash originale del dato
                        signature,    // La firma che vuoi verificare
                        HashAlgorithmName.SHA256, // L'algoritmo di hashing usato per l'hash
                        RSASignaturePadding.Pkcs1 // Il padding usato per la firma
                    );
                    Console.WriteLine($"\nVerifica della firma: {isValid}");
                }
                */
            }
            catch (CryptographicException ex)
            {
                Console.WriteLine($"\nErrore di crittografia: {ex.Message}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"\nSi è verificato un errore generico: {ex.Message}");
            }

        

#endif