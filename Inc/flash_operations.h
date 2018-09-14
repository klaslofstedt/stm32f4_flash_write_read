#ifndef FLASH_OPERATIONS_H
#define FLASH_OPERATIONS_H

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_5   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_7   /* End @ of user Flash area */

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

uint8_t Flash_Write();
volatile uint32_t Flash_Read(uint32_t addr);
uint8_t Flash_Check(uint32_t start_addr, uint32_t end_addr);
uint8_t Flash_WriteWord(uint32_t flash_data);
uint8_t Flash_EraseAll();
uint32_t Flash_GetEndAddr();

#endif
