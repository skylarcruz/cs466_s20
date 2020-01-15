# cs466_s20
Lab information and examples for CS466 Embedded Systems, Spring 2020

## If you want to use my pre-configured makefiles there are two required peer directories
Assuming that you let this repo install with a default name cs466_s19 there are two required directories that you need to install.
1. TivaDriver 
  * This is an library containing header files describing the processor and Tiva Board
  * It also contains a serise of higher level API's that we will be using in class later
2. FreeRTOS

Before you build your parent directoy will look like:
```
   ...
   cs466_s19/
   TivaDriver/
   FreeRTOSv10.2.1/
   ...
```
You can rename the directories or move them around but my provided makefiles will require modification each week.

for Linux (and probably OSx) you can use the TivaDriver tar file that I have put in .../cs466_s19/doc.

1. change your directory so that you are in ../cs466_s19 ir the parent of cs466_s19.
2. Run the command.
```
$ tar zxvf ./cs466_s19/doc/TivaDriver.tar.gz
```
It should create the peer TivaDriver directory.

## FreeRTOS is not used for lab01 but you can install it

1. change your directory so that you are in ../cs466_s19 ir the parent of cs466_s19.
2. Download FreeRTOS Version 10.2.1 from the following URL
   * https://sourceforge.net/projects/freertos
3. Unzip the distrubition to the peer directory
```
$ cd <parent directory>
$ unzip ~/Downloads/FreeRTOSv10.1.1.zip
```

