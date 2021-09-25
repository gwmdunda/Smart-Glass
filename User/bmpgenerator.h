
#define image_sizes  HORIZONTAL_PIXEL_SIZE * VERTICAL_PIXEL_SIZE
#define file_sizes  54 + 3 * image_sizes
#define dpi 100
#define ppm dpi*39.375


#pragma pack(push, 1)
typedef struct bitmap_file_header {
    unsigned char   bitmap_type[2];     // 2 bytes
    int             file_size;          // 4 bytes
    short           reserved1;          // 2 bytes
    short           reserved2;          // 2 bytes
    unsigned int    offset_bits;        // 4 bytes
} bitmap_file_header;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    unsigned int    size_header;        // 4 bytes
    unsigned int    width;              // 4 bytes
    unsigned int    height;             // 4 bytes
    short int       planes;             // 2 bytes
    short int       bit_count;          // 2 bytes
    unsigned int    compression;        // 4 bytes
    unsigned int    image_size;         // 4 bytes
    unsigned int    ppm_x;              // 4 bytes
    unsigned int    ppm_y;              // 4 bytes
    unsigned int    clr_used;           // 4 bytes
    unsigned int    clr_important;      // 4 bytes
} bitmap_image_header;
#pragma pack(pop)


#define red_mask  0x00ff0000          // Bit mask for the red channel
#define green_mask 0x0000ff00        // Bit mask for the green channel
#define blue_mask  0x000000ff         // Bit mask for the blue channel
#define alpha_mask  0xff000000        // Bit mask for the alpha channel
#define color_space_type 0x73524742  ; // Default "sRGB" (0x73524742)
static uint32_t unused[16] ;                // Unused data for sRGB color space



void generateBMP(const char* filname, FATFS* fs,FIL* image );

