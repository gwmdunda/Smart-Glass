#include "sdcard\ff.h"
#include "ov7725\bsp_ov7725.h"
#include <stdlib.h>
#include <string.h>
#include "bmpgenerator.h"
#include "ssd1306.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdarg.h>

char test[100];

void generateBMP(const char* filname, FATFS* fs, FIL* image)
{
	int i, j;
	uint16_t Camera_Data;
	uint8_t B8;
	uint8_t R8;
	uint8_t G8;
	//uint16_t zeropad = 0;
	UINT bw;
	bitmap_file_header bfh;
	bitmap_image_header bih;
	
	
	memcpy(&bfh.bitmap_type, "BM", 2);
	bfh.file_size       = file_sizes;
	bfh.reserved1       = 0;
	bfh.reserved2       = 0;
	bfh.offset_bits     = 0;
	
	bih.size_header     = sizeof(bih);
	bih.width           = HORIZONTAL_PIXEL_SIZE;
	bih.height          = VERTICAL_PIXEL_SIZE;
	bih.planes          = 1;
	bih.bit_count       = 24;
	bih.compression     = 0;
	bih.image_size      = 0;
	bih.ppm_x           = 0;
	bih.ppm_y           = 0;
	bih.clr_used        = 0;
	bih.clr_important   = 0;
	
	f_mount(fs,"",0);
	f_open(image, filname, FA_CREATE_ALWAYS|FA_WRITE);
	f_write(image,&bfh,sizeof(bfh),&bw);
	f_write(image,&bih,sizeof(bih),&bw);
	
	for(i = VERTICAL_PIXEL_SIZE; i >= 0; i--)
	{
		for(j = 0; j < HORIZONTAL_PIXEL_SIZE; j++)
		{
			READ_FIFO_PIXEL(Camera_Data);
			
			B8 = ( ((Camera_Data & 0xF800)>>11) * 527 + 23 ) >> 6;
			G8 = ( ((Camera_Data & 0x07E0)>>5) * 259 + 33 ) >> 6;
			R8 = ( (Camera_Data & 0x1F) * 527 + 23 ) >> 6;
		
			f_write(image,&B8,1,&bw);
			f_write(image,&G8,1,&bw);
			f_write(image,&R8,1,&bw);
		
			
				
		}
		//f_write(image,&zeropad,2,bw);
	}
	
		
	
	f_close(image);
}

