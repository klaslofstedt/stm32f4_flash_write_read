#include "stm32f4xx_hal.h"
#include "flash_operations.h"
//#include "uart_print.h"

/* Private variables ---------------------------------------------------------*/

#define DATA_32                 ((uint32_t)0xEEEEEEEE)
// TODO: error codes instead of just status 0 and 1
uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0, SectorError = 0;
__IO uint32_t data32 = 0, MemoryProgramStatus = 0;

uint32_t addr_write = FLASH_USER_START_ADDR;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

static uint32_t GetSector(uint32_t Address);

uint32_t Flash_GetEndAddr()
{
	return addr_write;
}

uint8_t Flash_EraseAll()
{
	uint8_t status = 1;
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();
	/* Erase the user Flash area
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Get the 1st sector to erase */
	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*
		 Error occurred while sector erase.
		 User can add here some code to deal with this error.
		 SectorError will contain the faulty sector and then to know the code error on this sector,
		 user can call function 'HAL_FLASH_GetError()'
		 */
		/*
		 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		 */
		status = 0;
		//UART_Print("Err1\n\r");
	}

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	HAL_FLASH_Lock();
	return status;
}

uint8_t Flash_WriteWord(uint32_t flash_data)
{
	uint8_t status = 1;
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	if (addr_write < FLASH_USER_END_ADDR)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr_write, flash_data) == HAL_OK)
		{
			if(!(Flash_Read(addr_write) == flash_data)){
				status = 0;
			}
			addr_write = addr_write + 4;
		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			/*
			 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
			 */
			status = 0;
		}
	}
	else{
		// TODO: EraseAll()?
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	/* Check if data is OK */
	//return Flash_Check(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);
	return status;
}

uint8_t Flash_Write()
{
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Get the 1st sector to erase */
	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*
		 Error occurred while sector erase.
		 User can add here some code to deal with this error.
		 SectorError will contain the faulty sector and then to know the code error on this sector,
		 user can call function 'HAL_FLASH_GetError()'
		 */
		/*
		 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		 */
		//UART_Print("Err3\n\r");
	}

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	/* Program the user Flash area word by word
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	Address = FLASH_USER_START_ADDR;

	while (Address < FLASH_USER_END_ADDR)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) == HAL_OK)
		{
			Address = Address + 4;
		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			/*
			 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
			 */
			//UART_Print("Err4\n\r");
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	/* Check if data is OK */
	return Flash_Check(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);
}

uint32_t Flash_Read(uint32_t addr)
{
	__IO uint32_t flash_word = *(__IO uint32_t*) addr;
	return flash_word;
}

uint8_t Flash_Check(uint32_t start_addr, uint32_t end_addr)
{
	/* Check if the programmed data is OK
	 MemoryProgramStatus = 0: data programmed correctly
	 MemoryProgramStatus != 0: number of words not programmed correctly ******/
	uint32_t addr = start_addr;
	MemoryProgramStatus = 0x0;

	while (addr < end_addr)
	{
		data32 = *(__IO uint32_t*) addr;

		if (data32 != DATA_32)
		{
			MemoryProgramStatus++;
		}

		addr = addr + 4;
	}

	/* Check if there is an issue to program data */
	if (MemoryProgramStatus == 0)
	{
		/* No error detected. Return 1*/
		return 1;
	}
	else
	{
		/* Error detected. */
		//UART_Print("Err5\n\r");
	}
	return 0;
}

/**
 * @brief  Gets the sector of a given address
 * @param  None
 * @retval The sector of a given address
 */
static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if ((Address < ADDR_FLASH_SECTOR_2)
			&& (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if ((Address < ADDR_FLASH_SECTOR_3)
			&& (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if ((Address < ADDR_FLASH_SECTOR_4)
			&& (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if ((Address < ADDR_FLASH_SECTOR_5)
			&& (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if ((Address < ADDR_FLASH_SECTOR_6)
			&& (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if ((Address < ADDR_FLASH_SECTOR_7)
			&& (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
	{
		sector = FLASH_SECTOR_7;
	}

	return sector;
}

