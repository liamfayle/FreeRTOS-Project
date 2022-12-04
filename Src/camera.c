/*
 * camera.c
 *
 * 	-- Liam Fayle modified on 2022-11-25
 *
 */



#include "camera.h"
#include "arducam.h"

#include "stlogo.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_cdc_if.h"

#include "picojpeg.h"


static uint32_t captureStart;
extern int myprintf(const char *format, ...);

static void camera_get_image();
BaseType_t write_fifo_to_buffer(uint32_t length);
unsigned char cameraReady;

#define min(a,b)    (((a) < (b)) ? (a) : (b))

uint8_t *ptr_picture = NULL;



void camera_setup(){

	cameraReady = pdFALSE;
	/**
	 * Detect and initialize the Arduchip interface.
	 * Ensure that the OV5642 is powered on.
	 * Detect and initialize the OV5642 sensor chip.
	 */
	if (   arduchip_detect()
		&& arducam_exit_standby()
		&& ov5642_detect()
	) {

		osDelay(100);

		if (!ov5642_configure()) {
			myprintf("camera_task: ov5642 configure failed\n\r");
			return;
		} else {
			myprintf("camera: setup complete\n\r");
			cameraReady = pdTRUE;
			osDelay(100);
		}
	} else {
		myprintf("camera: setup failed\n\r");
	}
}

/**
 * Capture an image from the camera.
 */
void camera_initiate_capture(){

	uint8_t done = 0;

	if (!cameraReady) {
		myprintf("camera: set up camera before capture\n\r");
	}

	/* Initiate an image capture. */
	if (!arduchip_start_capture()) {
		myprintf("camera: initiate capture failed\n\r");
		return;
	}

	/* wait for capture to be done */
	captureStart = (uint32_t)xTaskGetTickCount();
	while(!arduchip_capture_done(&done) || !done){

		if ((xTaskGetTickCount() - captureStart) >= CAPTURE_TIMEOUT) {
			myprintf("camera: capture timeout\n\r");
			return;
		}
	}

	camera_get_image();

	return;

}

void camera_get_image(){

	/* Determine the FIFO buffer length. */
	uint32_t length = 0;
	if (arduchip_fifo_length(&length) == pdTRUE) {
		write_fifo_to_buffer(length);
	} else {
		myprintf("camera: get fifo length failed\n\r");
	}

	return;
}

uint8_t fifoBuffer[BURST_READ_LENGTH];
size_t count = 0;
FILE *g_pInFile;
uint g_nInFileSize;
uint g_nInFileOfs;



unsigned char pjpeg_need_bytes_callback(unsigned char* pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data)
{
   uint n;

   n = min(g_nInFileSize - g_nInFileOfs, buf_size);
   if (n && (fread(pBuf, 1, n, g_pInFile) != n))
      return PJPG_STREAM_READ_ERROR;
   *pBytes_actually_read = (unsigned char)(n);
   g_nInFileOfs += n;
   return 0;
}

typedef struct _JPEG_Info
{
	int loadStatus;
	int loadedWidth;
	int loadedHeight;
	int actualWidth;
	int actualHeight;
	pjpeg_scan_type_t scanType;
	int colorComps;

} JPEG_Info;

JPEG_Info pjpeg_load_from_file(FILE *fp, int reduce)
{
	JPEG_Info retVal;
	retVal.loadStatus = 0;
	retVal.loadedWidth = 0;
	retVal.loadedHeight = 0;
	retVal.actualWidth = 0;
	retVal.actualHeight = 0;
	retVal.colorComps = 0;
	retVal.scanType = PJPG_GRAYSCALE;

	pjpeg_image_info_t image_info;
	int mcu_x = 0;
	int mcu_y = 0;

	uint8_t status;
	uint decoded_width, decoded_height;

	g_pInFile = fp;
	if (!g_pInFile)
	{
		retVal.loadStatus = 1;
		return retVal;
	}

	g_nInFileOfs = 0;

	fseek(g_pInFile, 0, SEEK_END);
	g_nInFileSize = ftell(g_pInFile);
	fseek(g_pInFile, 0, SEEK_SET);

	status = pjpeg_decode_init(&image_info, pjpeg_need_bytes_callback, NULL, (unsigned char)reduce);

	if (status)
	{
		retVal.loadStatus = status;
		fclose(g_pInFile);
		return retVal;
	}

	retVal.scanType = image_info.m_scanType;

	// In reduce mode output 1 pixel per 8x8 block.
	decoded_width = reduce ? (image_info.m_MCUSPerRow * image_info.m_MCUWidth) / 8 : image_info.m_width;
	decoded_height = reduce ? (image_info.m_MCUSPerCol * image_info.m_MCUHeight) / 8 : image_info.m_height;

	// assigned image information
	retVal.actualWidth = image_info.m_width;
	retVal.actualHeight = image_info.m_height;
	retVal.loadedWidth = decoded_width;
	retVal.loadedHeight = decoded_height;
	retVal.colorComps = image_info.m_comps;

	for ( ; ; )
	{
		status = pjpeg_decode_mcu();

		if (status)
		{
			if (status != PJPG_NO_MORE_BLOCKS)
			{
				retVal.loadStatus = status;
				fclose(g_pInFile);
				return retVal;
			}

			break;
		}

		if (mcu_y >= image_info.m_MCUSPerCol)
		{
			retVal.loadStatus = 2;
			fclose(g_pInFile);
			return retVal;
		}

		if (reduce)
		{
			// In reduce mode, only the first pixel of each 8x8 block is valid.
			// m_comps: color component (1 for grayscale, 3 for RGB)
			/*uint y, x;
			for (y = 0; y < col_blocks_per_mcu; y++)
			{
				uint src_ofs = (y * 128U);

				for (x = 0; x < row_blocks_per_mcu; x++)
				{
					uint lcd_x = decoded_width - mcu_x * row_blocks_per_mcu + x - 1;
					uint lcd_y = mcu_y * col_blocks_per_mcu + y;

					src_ofs += 64;
				}
			}*/
			myprintf("Reduce not implemented\n\r");
		}
		else
		{
			// Copy MCU's pixel blocks into the destination bitmap.
			uint y, x;

			for (y = 0; y < image_info.m_MCUHeight; y += 8)
			{
				const int by_limit = min(8, image_info.m_height - (mcu_y * image_info.m_MCUHeight + y));

				for (x = 0; x < image_info.m_MCUWidth; x += 8)
				{
					// Compute source byte offset of the block in the decoder's MCU buffer.
					uint src_ofs = (x * 8U) + (y * 16U);
					const uint8_t *pSrcR = image_info.m_pMCUBufR + src_ofs;
					const uint8_t *pSrcG = image_info.m_pMCUBufG + src_ofs;
					const uint8_t *pSrcB = image_info.m_pMCUBufB + src_ofs;

					const int bx_limit = min(8, image_info.m_width - (mcu_x * image_info.m_MCUWidth + x));

					int bx, by;
					for (by = 0; by < by_limit; by++)
					{
						for (bx = 0; bx < bx_limit; bx++)
						{
							uint lcd_x = image_info.m_width - (mcu_x * image_info.m_MCUWidth + x + bx) - 1;
							uint lcd_y = mcu_y * image_info.m_MCUHeight + y + by;


							uint32_t rgb = (0xFF << 24) | (*pSrcR++ << 16) | (*pSrcG++ << 8) | (*pSrcB++);

							BSP_LCD_DrawPixel(lcd_x, 50 + lcd_y, rgb);
						}

						pSrcR += (8 - bx_limit);
						pSrcG += (8 - bx_limit);
						pSrcB += (8 - bx_limit);
					}
				}
			}
		}

		mcu_x++;
		if (mcu_x == image_info.m_MCUSPerRow)
		{
			mcu_x = 0;
			mcu_y++;
		}
	}

	fclose(g_pInFile);

	retVal.loadStatus = 0;
	return retVal;
}



BaseType_t
write_fifo_to_buffer(uint32_t length)
{
#if 1	// display captured image

	/* Write the FIFO contents to disk. */
	uint16_t chunk = 0;

	free(ptr_picture);
	// jpeg pic size
	unsigned int jpeg_size = length*sizeof(uint8_t);
	// allocate memory to store jpeg picture
	if((ptr_picture = malloc(jpeg_size)) == NULL){
		myprintf("camera: ran out of memory\n\r");
	}

	FILE *fp = fmemopen(ptr_picture, jpeg_size, "a");

	for (uint16_t i = 0; length > 0; ++i) {

		chunk = MIN(length, BURST_READ_LENGTH);
		arduchip_burst_read(fifoBuffer, chunk);
		length -= chunk;

		fwrite(fifoBuffer, sizeof(uint8_t), chunk, fp);

	}

	fclose(fp);

	fp = fmemopen(ptr_picture, jpeg_size, "rb");

	JPEG_Info image = pjpeg_load_from_file(fp, 0);

	if (image.loadStatus) {
		myprintf("image failed to load\n\r");
	}


#else
	// test image: make sure to build the project with -Og to show this static .bmp image
	// 		project properties -> C/C++ Build -> Settings -> Optimization | Optimize for debugging (-Og)
    BSP_LCD_DrawBitmap(80, 180, (uint8_t *)stlogo);
#endif
    osDelay(500);

	return pdTRUE;
}


