/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_ADDRESS 0x50  // Dirección del dispositivo esclavo (I2C5)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c5;

IPCC_HandleTypeDef hipcc;

/* USER CODE BEGIN PV */
uint8_t contador_descendente = 255;
uint8_t contador_ascendente = 0;
uint8_t rx_buffer[1];  // Buffer para recibir datos
HAL_StatusTypeDef status;
uint32_t last_time = 0;
uint8_t aTxBuffer[] = "STM32MP1 - I2C1 Master OK";
uint8_t aRxBuffer[sizeof(aTxBuffer)];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C5_Init(void);
static void MX_IPCC_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */
void I2C5_Slave_Task_Init(void);
void I2C1_Master_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();
  }
  else
  {
    /* IPCC initialisation */
    MX_IPCC_Init();
    /* OpenAmp initialisation ---------------------------------*/
    MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C5_Init();
  /* USER CODE BEGIN 2 */
  contador_descendente = 255;
  contador_ascendente = 0;
  last_time = HAL_GetTick();
  
  // Habilitar modo esclavo en I2C5
  HAL_I2C_EnableListen_IT(&hi2c5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Verificar si han pasado 200ms
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_time >= 200)
    {
      last_time = current_time;
      
      // 1. I2C1 (Maestro) envía contador descendente al esclavo
      status = HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS << 1, &contador_descendente, 1, 100);
      
      // 2. Esperamos un poco para dar tiempo al esclavo a procesar
      HAL_Delay(5);
      
      // 3. I2C1 (Maestro) solicita contador ascendente del esclavo
      status = HAL_I2C_Master_Receive(&hi2c1, SLAVE_ADDRESS << 1, rx_buffer, 1, 100);
      
      // 4. Actualizamos contador solo si la comunicación fue exitosa
      if (status == HAL_OK) {
        contador_descendente--;
      }
      
      // Si hay error en el bus, reiniciamos el periférico
      if (status != HAL_OK) {
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
          HAL_I2C_DeInit(&hi2c1);
          HAL_I2C_Init(&hi2c1);
        }
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 6660;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C5_Init(void)
{

  /* USER CODE BEGIN I2C5_Init 0 */

  /* USER CODE END I2C5_Init 0 */

  /* USER CODE BEGIN I2C5_Init 1 */

  /* USER CODE END I2C5_Init 1 */
  hi2c5.Instance = I2C5;
  hi2c5.Init.Timing = 0x40505874;
  hi2c5.Init.OwnAddress1 = SLAVE_ADDRESS << 1; // Dirección propia como esclavo
  hi2c5.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c5.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c5.Init.OwnAddress2 = 0;
  hi2c5.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c5.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c5.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c5, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C5_Init 2 */

  /* USER CODE END I2C5_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  I2C slave receive complete callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C5)
  {
    // El maestro ha terminado de enviar datos
    // No necesitamos hacer nada especial aquí, el hardware ya ha almacenado
    // el dato en rx_buffer
    
    // Reactivar el modo escucha para futuras comunicaciones
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

/**
  * @brief  I2C slave transmit complete callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C5)
  {
    // El esclavo ha terminado de enviar datos al maestro
    // Incrementamos el contador ascendente para la próxima solicitud
    contador_ascendente++;
    
    // Reactivar el modo escucha para futuras comunicaciones
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

/**
  * @brief  I2C slave address match callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  TransferDirection: Master request direction (Write/Read)
  * @param  AddrMatchCode: Address match code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (hi2c->Instance == I2C5)
  {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) // Maestro transmite (esclavo recibe)
    {
      // Preparar para recibir datos del maestro
      HAL_I2C_Slave_Receive_IT(hi2c, rx_buffer, 1);
    }
    else // Maestro recibe (esclavo transmite)
    {
      // Enviar contador ascendente al maestro
      HAL_I2C_Slave_Transmit_IT(hi2c, &contador_ascendente, 1);
    }
  }
}

/**
  * @brief  I2C error callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C5)
  {
    // En caso de error en el esclavo, reiniciamos el periférico
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
    HAL_I2C_EnableListen_IT(hi2c);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/* Funcion de Tarea del Esclavo (se llama en el main.c antes del loop principal) */
void I2C5_Slave_Task_Init(void)
{
    // Inicia al I2C5 en modo ESCLAVO para recibir 'size' bytes.
    // La funcion es no-bloqueante y espera la llamada del maestro (se necesita un handler de interrupción o DMA en un caso real).
    // Para esta prueba simple (polling-simulado), usaremos la version bloqueante:
    
    if (HAL_I2C_Slave_Receive(&hi2c5, aRxBuffer, sizeof(aTxBuffer), 5000) != HAL_OK)
    {
        // Si hay un error, el LED de error se activa (Asumiendo LED LD7 en PH7)
        // La implementación real requeriría un loop infinito esperando comandos.
        // Simulando manejo de error:
        //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET); // Asumiendo LED LD7 (Error)
    }
    else
    {
        // ¡ÉXITO! Dato recibido.
        // 1. Verificar si el mensaje recibido coincide con el enviado (no implementado aquí)
        // 2. Encender un LED (ej. LD5/LD6) para indicar recepción exitosa (Asumiendo LED LD6 en PA13)
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET); // Asumiendo LED LD6 (Éxito)
    }
}

/* Funcion de Tarea del Maestro (se llama en el loop principal while(1)) */
void I2C1_Master_Task(void)
{
    // El Maestro envía el buffer 'aTxBuffer' a la dirección del esclavo I2C_SLAVE_ADDRESS.
    // HAL_MAX_DELAY indica que la función es bloqueante.

    if (HAL_I2C_Master_Transmit(&hi2c1, 
                                I2C_SLAVE_ADDRESS, 
                                aTxBuffer, 
                                sizeof(aTxBuffer), 
                                1000) != HAL_OK) // Timeout de 1000ms
    {
        // Error de Transmisión (No ACK del esclavo o error de bus)
        
        // Comprobar si es un error de No-ACK (esclavo no respondió a la dirección)
        if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
        {
            // El esclavo no reconoció la dirección. (LD7 ON - Rojo/Naranja)
            //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);
        }
        else
        {
            // Otro error de bus.
            //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);
        }
    }
    else
    {
        // Transmisión completada exitosamente. (LD5 ON - Verde)
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
    }
    
    // Espera para evitar retransmisiones inmediatas
    HAL_Delay(5000); 
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
