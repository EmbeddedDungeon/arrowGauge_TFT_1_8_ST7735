#include "SDFlash.h"

#include "stm32f4xx.h"  // Убедитесь, что используете правильный заголовочный файл для вашей серии

void SPI2_Init(void) {
    // Включаем тактирование для GPIOB и SPI2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Включаем тактирование для GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;   // Включаем тактирование для SPI2

    // Настраиваем пины GPIOB 13 (SCK), 14 (MISO) и 15 (MOSI) в режим альтернативной функции (AF5 для SPI2)
    GPIOB->MODER &= ~(GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk);
    GPIOB->MODER |= (GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1);  // Устанавливаем режим альтернативной функции

    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL13_Msk | GPIO_AFRH_AFSEL14_Msk | GPIO_AFRH_AFSEL15_Msk);
    GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFSEL13_Pos) | (5 << GPIO_AFRH_AFSEL14_Pos) | (5 << GPIO_AFRH_AFSEL15_Pos); // Альтернативная функция AF5 для SPI2

    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15);  // Высокая скорость
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD13_Msk | GPIO_PUPDR_PUPD14_Msk | GPIO_PUPDR_PUPD15_Msk);  // Без подтяжки
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15);  // Пуш-пулл

    // Настройка SPI2
    SPI2->CR1 = 0;  // Сбросим регистр управления
    SPI2->CR1 |= SPI_CR1_MSTR;  // Устанавливаем режим ведущего (Master)
    SPI2->CR1 |= SPI_CR1_BR_2;          // Установите делитель 8 (10.5 MHz)
    //SPI2->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;  // Настраиваем скорость передачи (Baud Rate)
    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // Программное управление SS (SSM) и внутренний сигнал SS (SSI)

    // Включаем прерывания для TXE и RXNE
    SPI2->CR2 |= SPI_CR2_TXEIE | SPI_CR2_RXNEIE;

    // Включаем SPI2
    SPI2->CR1 |= SPI_CR1_SPE;

    // Включаем прерывания в NVIC
    NVIC_EnableIRQ(SPI2_IRQn);
}


void SPI2_SendByte(uint8_t byte) {
    txBuffer[txHead] = byte;
    txHead = (txHead + 1) % sizeof(txBuffer);

    if ((txHead == txTail) && !(SPI2->CR2 & SPI_CR2_TXEIE)) {
        SPI2->CR2 |= SPI_CR2_TXEIE;  // Включаем прерывание TXE только если буфер был пуст
    }
}


uint8_t SPI2_IsDataAvailable(void) {
    return rxHead != rxTail;
}

uint8_t SPI2_GetReceivedByte(void) {
    if (SPI2_IsDataAvailable()) {
        uint8_t byte = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % sizeof(rxBuffer);
        return byte;
    }
    return 0;  // Или другой код ошибки
}

