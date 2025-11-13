@echo Controllare che il file BOOT_Vxx.bin sia presente nella directory TARGET\Exe
cd %1
dir > output.txt

..\Win\CheckTag\bin\Debug\CheckTagGD.exe  
