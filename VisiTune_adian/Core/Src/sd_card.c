/*
 * sd_card.c
 *
 *  Created on: Nov 24, 2025
 *      Author: nick
 */

#include "sd_card.h"

void read_sound_file(const TCHAR* file_name, uint16_t *pData, uint32_t dataSize) {

			memset(pData, 0, sizeof(pData));

		  //some variables for FatFs
		  FATFS FatFs; 	//Fatfs handle
		  FIL fil; 		//File handle
		  FRESULT fres; //Result after operations

		  //Open the file system
		  fres = f_mount(&FatFs, "", 1); //1=mount now
		  if(fres != FR_OK) {
			  while(1);
		  }

		  //Let's get some statistics from the SD card
		  DWORD free_clusters, free_sectors, total_sectors;

		  FATFS* getFreeFs;

		  fres = f_getfree("", &free_clusters, &getFreeFs);
		  if(fres != FR_OK) {
			  while(1);
		  }

		  //Formula comes from ChaN's documentation
		  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
		  free_sectors = free_clusters * getFreeFs->csize;

		  //Now let's try to open file "test.txt"
		  fres = f_open(&fil, file_name, FA_READ);
		  if(fres != FR_OK) {
			  while(1);
		  }

		  //We can either use f_read OR f_gets to get data out of files
		  //f_gets is a wrapper on f_read that does some string formatting for us
		  UINT bytesRead = 0;
		  FRESULT rres = f_read(&fil, pData, dataSize, &bytesRead);
		  if(rres != FR_OK) {
			  while(1);
		  }

		  //Be a tidy kiwi - don't forget to close your file!
		  f_close(&fil);
}



