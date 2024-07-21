#include "SDFlash.h"

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
    SPI2->CR1 |= SPI_CR1_BR;    // Настраиваем скорость передачи (Baud Rate)
    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // Программное управление SS (SSM) и внутренний сигнал SS (SSI)
    SPI2->CR1 |= SPI_CR1_SPE;   // Включаем SPI2
}

void SPI2_SendByte(uint8_t byte) {
    while (!(SPI2->SR & SPI_SR_TXE));  // Ждем пока буфер передатчика не освободится
    SPI2->DR = byte;
    while (!(SPI2->SR & SPI_SR_RXNE)); // Ждем окончания передачи
    (void)SPI2->DR;                    // Читаем принятый байт для очистки флага
}

uint8_t SPI2_ReceiveByte(void) {
    while (!(SPI2->SR & SPI_SR_TXE));  // Ждем пока буфер передатчика не освободится
    SPI2->DR = 0xFF;                   // Отправляем пустой байт
    while (!(SPI2->SR & SPI_SR_RXNE)); // Ждем окончания приема
    return SPI2->DR;                   // Читаем принятый байт
}

uint8_t SPI2_TransferByte(uint8_t byte) {
    while (!(SPI2->SR & SPI_SR_TXE));  // Ждем пока буфер передатчика не освободится
    SPI2->DR = byte;                   // Отправляем байт
    while (!(SPI2->SR & SPI_SR_RXNE)); // Ждем окончания приема
    return SPI2->DR;                   // Возвращаем принятый байт
}

char SPI2_LoopbackTest(void) {
    uint8_t data = 0xAA;  // Данные для отправки
    uint8_t received;

    received = SPI2_TransferByte(data);

    if (received == data) {
        return 'y';
    	// Успешно, данные совпадают
    } else {
    	return 'n';
        // Ошибка, данные не совпадают
    }
}
