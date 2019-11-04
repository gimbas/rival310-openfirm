#include "uart.h"

// Define the UART console
void _putchar(char ch)
{
    uart1_write_byte((uint8_t)ch);
}


static volatile uint8_t *pubUART1DMABuffer = NULL;
static volatile uint8_t *pubUART1FIFO = NULL;
static volatile uint16_t usUART1FIFOWritePos, usUART1FIFOReadPos;

void _usart1_isr()
{
    if(USART1->SR & USART_SR_IDLE)
    {
        REG_DISCARD(&USART1->SR); // Read SR & DR to clear interrupt flag
        REG_DISCARD(&USART1->DR);

        uint32_t ulSize = (UART1_DMA_RX_BUFFER_SIZE >> 1) - ((DMA1_Channel5->CNDTR - 1) % (UART1_DMA_RX_BUFFER_SIZE >> 1)) - 1;

        if(ulSize)
        {
            DMA1_Channel5->CCR &= ~DMA_CCR5_EN; // Disable DMA channel

            volatile uint8_t *pubDMABufferReadPos = (DMA1_Channel5->CNDTR > (UART1_DMA_RX_BUFFER_SIZE >> 1)) ? pubUART1DMABuffer : pubUART1DMABuffer + (UART1_DMA_RX_BUFFER_SIZE >> 1);

            while(ulSize--)
            {
                pubUART1FIFO[usUART1FIFOWritePos++] = *pubDMABufferReadPos++;

                if(usUART1FIFOWritePos >= UART1_FIFO_SIZE)
                    usUART1FIFOWritePos = 0;
            }

            DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5; // Clear all interrupt flags otherwise DMA won't start again
            DMA1_Channel5->CNDTR = UART1_DMA_RX_BUFFER_SIZE;
            DMA1_Channel5->CMAR = (uint32_t)pubUART1DMABuffer;
            DMA1_Channel5->CCR |= DMA_CCR5_EN; // Enable DMA channel
        }
    }
}
void _dma1_channel5_isr()
{
    volatile uint8_t *pubDMABufferReadPos = 0;

    if(DMA1->ISR & DMA_ISR_HTIF5)
    {
        DMA1->IFCR = DMA_IFCR_CHTIF5; // Clear Half-transfer flag

        pubDMABufferReadPos = pubUART1DMABuffer;
    }
    else if(DMA1->ISR & DMA_ISR_TCIF5)
    {
        DMA1->IFCR = DMA_IFCR_CTCIF5; // Clear Transfer complete flag

        pubDMABufferReadPos = pubUART1DMABuffer + (UART1_DMA_RX_BUFFER_SIZE >> 1);
    }

    if(pubDMABufferReadPos)
    {
        uint32_t ulSize = (UART1_DMA_RX_BUFFER_SIZE >> 1);

        while(ulSize--)
        {
            pubUART1FIFO[usUART1FIFOWritePos++] = *pubDMABufferReadPos++;

            if(usUART1FIFOWritePos >= UART1_FIFO_SIZE)
                usUART1FIFOWritePos = 0;
        }
    }
}
void uart1_init(uint32_t ulBaud)
{
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable USART peripheral clock

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable DMA1 peripheral clock

    free((uint8_t *)pubUART1DMABuffer);
    free((uint8_t *)pubUART1FIFO);

    pubUART1DMABuffer = (volatile uint8_t *)malloc(UART1_DMA_RX_BUFFER_SIZE);

    if(!pubUART1DMABuffer)
        return;

    memset((uint8_t *)pubUART1DMABuffer, 0, UART1_DMA_RX_BUFFER_SIZE);

    pubUART1FIFO = (volatile uint8_t *)malloc(UART1_FIFO_SIZE);

    if(!pubUART1FIFO)
    {
        free((void *)pubUART1DMABuffer);

        return;
    }

    memset((uint8_t *)pubUART1FIFO, 0, UART1_FIFO_SIZE);

    usUART1FIFOWritePos = 0;
    usUART1FIFOReadPos = 0;

    USART1->CR1 = USART_CR1_UE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
    USART1->CR2 = 0x0000;
    USART1->CR3 = USART_CR3_DMAR;

    uint32_t ulIntDiv = ((25 * APB2_CLOCK_FREQ) / (4 * ulBaud));
    uint32_t ulRegValue = (ulIntDiv / 100) << 4;
    uint32_t ulFlDiv = ulIntDiv - (100 * (ulRegValue >> 4));
    ulRegValue |= ((((ulFlDiv * 16) + 50) / 100)) & 0x0F;

    USART1->BRR = ulRegValue;

    IRQ_SET_PRIO(USART1_IRQn, 6, 1);
    IRQ_CLEAR(USART1_IRQn);
    IRQ_ENABLE(USART1_IRQn);

    DMA1_Channel5->CNDTR = UART1_DMA_RX_BUFFER_SIZE;
    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel5->CMAR = (uint32_t)pubUART1DMABuffer;
    DMA1_Channel5->CCR = (1 << 12) | DMA_CCR5_MINC | DMA_CCR5_CIRC | DMA_CCR5_HTIE | DMA_CCR5_TCIE | DMA_CCR5_EN; // Medium priority, Memory increment, Circular mode, Half-transfer interrupt, Transfer Complete interrupt, Enable

    IRQ_SET_PRIO(DMA1_Channel5_IRQn, 6, 0);
    IRQ_CLEAR(DMA1_Channel5_IRQn);
    IRQ_ENABLE(DMA1_Channel5_IRQn);
}
void uart1_write_byte(const uint8_t ubData)
{
    while(!(USART1->SR & USART_SR_TXE));

    USART1->DR = ubData;
}
uint8_t uart1_read_byte()
{
    if(!uart1_available())
        return 0;

    uint8_t ubData;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ubData = pubUART1FIFO[usUART1FIFOReadPos++];

        if(usUART1FIFOReadPos >= UART1_FIFO_SIZE)
            usUART1FIFOReadPos = 0;
    }

    return ubData;
}
uint32_t uart1_available()
{
    return (UART1_FIFO_SIZE + usUART1FIFOWritePos - usUART1FIFOReadPos) % UART1_FIFO_SIZE;
}
void uart1_flush()
{
    usUART1FIFOReadPos = usUART1FIFOWritePos = 0;
}

static volatile uint8_t *pubUART2DMABuffer = NULL;
static volatile uint8_t *pubUART2FIFO = NULL;
static volatile uint16_t usUART2FIFOWritePos, usUART2FIFOReadPos;

void _usart2_isr()
{
    if(USART2->SR & USART_SR_IDLE)
    {
        REG_DISCARD(&USART2->SR); // Read SR & DR to clear interrupt flag
        REG_DISCARD(&USART2->DR);

        uint32_t ulSize = (UART2_DMA_RX_BUFFER_SIZE >> 1) - ((DMA1_Channel6->CNDTR - 1) % (UART2_DMA_RX_BUFFER_SIZE >> 1)) - 1;

        if(ulSize)
        {
            DMA1_Channel6->CCR &= ~DMA_CCR6_EN; // Disable DMA channel

            volatile uint8_t *pubDMABufferReadPos = (DMA1_Channel6->CNDTR > (UART2_DMA_RX_BUFFER_SIZE >> 1)) ? pubUART2DMABuffer : (pubUART2DMABuffer + (UART2_DMA_RX_BUFFER_SIZE >> 1));

            while(ulSize--)
            {
                pubUART2FIFO[usUART2FIFOWritePos++] = *pubDMABufferReadPos++;

                if(usUART2FIFOWritePos >= UART2_FIFO_SIZE)
                    usUART2FIFOWritePos = 0;
            }

            DMA1->IFCR = DMA_IFCR_CGIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTEIF6; // Clear all interrupt flags otherwise DMA won't start again
            DMA1_Channel6->CNDTR = UART2_DMA_RX_BUFFER_SIZE;
            DMA1_Channel6->CMAR = (uint32_t)pubUART2DMABuffer;
            DMA1_Channel6->CCR |= DMA_CCR6_EN; // Enable DMA channel
        }
    }
}
void _dma1_channel6_isr()
{
    volatile uint8_t *pubDMABufferReadPos = 0;

    if(DMA1->ISR & DMA_ISR_HTIF6)
    {
        DMA1->IFCR = DMA_IFCR_CHTIF6; // Clear Half-transfer flag

        pubDMABufferReadPos = pubUART2DMABuffer;
    }
    else if(DMA1->ISR & DMA_ISR_TCIF6)
    {
        DMA1->IFCR = DMA_IFCR_CTCIF6; // Clear Transfer complete flag

        pubDMABufferReadPos = pubUART2DMABuffer + (UART2_DMA_RX_BUFFER_SIZE >> 1);
    }

    if(pubDMABufferReadPos)
    {
        uint32_t ulSize = (UART2_DMA_RX_BUFFER_SIZE >> 1);

        while(ulSize--)
        {
            pubUART2FIFO[usUART2FIFOWritePos++] = *pubDMABufferReadPos++;

            if(usUART2FIFOWritePos >= UART2_FIFO_SIZE)
                usUART2FIFOWritePos = 0;
        }
    }
}
void uart2_init(uint32_t ulBaud)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART peripheral clock

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable DMA1 peripheral clock

    free((uint8_t *)pubUART2DMABuffer);
    free((uint8_t *)pubUART2FIFO);

    pubUART2DMABuffer = (volatile uint8_t *)malloc(UART2_DMA_RX_BUFFER_SIZE);

    if(!pubUART2DMABuffer)
        return;

    memset((uint8_t *)pubUART2DMABuffer, 0, UART2_DMA_RX_BUFFER_SIZE);

    pubUART2FIFO = (volatile uint8_t *)malloc(UART2_FIFO_SIZE);

    if(!pubUART2FIFO)
    {
        free((void *)pubUART2DMABuffer);

        return;
    }

    memset((uint8_t *)pubUART2FIFO, 0, UART2_FIFO_SIZE);

    usUART2FIFOWritePos = 0;
    usUART2FIFOReadPos = 0;

    USART2->CR1 = USART_CR1_UE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
    USART2->CR2 = 0x0000;
    USART2->CR3 = USART_CR3_DMAR;

    uint32_t ulIntDiv = ((25 * APB1_CLOCK_FREQ) / (4 * ulBaud));
    uint32_t ulRegValue = (ulIntDiv / 100) << 4;
    uint32_t ulFlDiv = ulIntDiv - (100 * (ulRegValue >> 4));
    ulRegValue |= ((((ulFlDiv * 16) + 50) / 100)) & 0x0F;

    USART2->BRR = ulRegValue;

    IRQ_SET_PRIO(USART2_IRQn, 6, 1);
    IRQ_CLEAR(USART2_IRQn);
    IRQ_ENABLE(USART2_IRQn);

    DMA1_Channel6->CNDTR = UART2_DMA_RX_BUFFER_SIZE;
    DMA1_Channel6->CPAR = (uint32_t)&USART2->DR;
    DMA1_Channel6->CMAR = (uint32_t)pubUART2DMABuffer;
    DMA1_Channel6->CCR = (1 << 12) | DMA_CCR6_MINC | DMA_CCR6_CIRC | DMA_CCR6_HTIE | DMA_CCR6_TCIE | DMA_CCR6_EN; // Medium priority, Memory increment, Circular mode, Half-transfer interrupt, Transfer Complete interrupt, Enable

    IRQ_SET_PRIO(DMA1_Channel6_IRQn, 6, 0);
    IRQ_CLEAR(DMA1_Channel6_IRQn);
    IRQ_ENABLE(DMA1_Channel6_IRQn);
}
void uart2_write_byte(const uint8_t ubData)
{
    while(!(USART2->SR & USART_SR_TXE));

    USART2->DR = ubData;
}
uint8_t uart2_read_byte()
{
    if(!uart2_available())
        return 0;

    uint8_t ubData;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ubData = pubUART2FIFO[usUART2FIFOReadPos++];

        if(usUART2FIFOReadPos >= UART2_FIFO_SIZE)
            usUART2FIFOReadPos = 0;
    }

    return ubData;
}
uint32_t uart2_available()
{
    return (UART2_FIFO_SIZE + usUART2FIFOWritePos - usUART2FIFOReadPos) % UART2_FIFO_SIZE;
}
void uart2_flush()
{
    usUART2FIFOReadPos = usUART2FIFOWritePos = 0;
}

static volatile uint8_t *pubUART3DMABuffer = NULL;
static volatile uint8_t *pubUART3FIFO = NULL;
static volatile uint16_t usUART3FIFOWritePos, usUART3FIFOReadPos;

void _usart3_isr()
{
    if(USART3->SR & USART_SR_IDLE)
    {
        REG_DISCARD(&USART3->SR); // Read SR & DR to clear interrupt flag
        REG_DISCARD(&USART3->DR);

        uint32_t ulSize = (UART3_DMA_RX_BUFFER_SIZE >> 1) - ((DMA1_Channel3->CNDTR - 1) % (UART3_DMA_RX_BUFFER_SIZE >> 1)) - 1;

        if(ulSize)
        {
            DMA1_Channel3->CCR &= ~DMA_CCR3_EN; // Disable DMA channel

            volatile uint8_t *pubDMABufferReadPos = (DMA1_Channel3->CNDTR > (UART3_DMA_RX_BUFFER_SIZE >> 1)) ? pubUART3DMABuffer : (pubUART3DMABuffer + (UART3_DMA_RX_BUFFER_SIZE >> 1));

            while(ulSize--)
            {
                pubUART3FIFO[usUART3FIFOWritePos++] = *pubDMABufferReadPos++;

                if(usUART3FIFOWritePos >= UART3_FIFO_SIZE)
                    usUART3FIFOWritePos = 0;
            }

            DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3; // Clear all interrupt flags otherwise DMA won't start again
            DMA1_Channel3->CNDTR = UART3_DMA_RX_BUFFER_SIZE;
            DMA1_Channel3->CMAR = (uint32_t)pubUART3DMABuffer;
            DMA1_Channel3->CCR |= DMA_CCR3_EN; // Enable DMA channel
        }
    }
}
void _dma1_channel3_isr()
{
    volatile uint8_t *pubDMABufferReadPos = 0;

    if(DMA1->ISR & DMA_ISR_HTIF3)
    {
        DMA1->IFCR = DMA_IFCR_CHTIF3; // Clear Half-transfer flag

        pubDMABufferReadPos = pubUART3DMABuffer;
    }
    else if(DMA1->ISR & DMA_ISR_TCIF3)
    {
        DMA1->IFCR = DMA_IFCR_CTCIF3; // Clear Transfer complete flag

        pubDMABufferReadPos = pubUART3DMABuffer + (UART3_DMA_RX_BUFFER_SIZE >> 1);
    }

    if(pubDMABufferReadPos)
    {
        uint32_t ulSize = (UART3_DMA_RX_BUFFER_SIZE >> 1);

        while(ulSize--)
        {
            pubUART3FIFO[usUART3FIFOWritePos++] = *pubDMABufferReadPos++;

            if(usUART3FIFOWritePos >= UART3_FIFO_SIZE)
                usUART3FIFOWritePos = 0;
        }
    }
}
void uart3_init(uint32_t ulBaud)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART peripheral clock

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable DMA1 peripheral clock

    free((uint8_t *)pubUART3DMABuffer);
    free((uint8_t *)pubUART3FIFO);

    pubUART3DMABuffer = (volatile uint8_t *)malloc(UART3_DMA_RX_BUFFER_SIZE);

    if(!pubUART3DMABuffer)
        return;

    memset((uint8_t *)pubUART3DMABuffer, 0, UART3_DMA_RX_BUFFER_SIZE);

    pubUART3FIFO = (volatile uint8_t *)malloc(UART3_FIFO_SIZE);

    if(!pubUART3FIFO)
    {
        free((void *)pubUART3DMABuffer);

        return;
    }

    memset((uint8_t *)pubUART3FIFO, 0, UART3_FIFO_SIZE);

    usUART3FIFOWritePos = 0;
    usUART3FIFOReadPos = 0;

    USART3->CR1 = USART_CR1_UE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
    USART3->CR2 = 0x0000;
    USART3->CR3 = USART_CR3_DMAR;

    uint32_t ulIntDiv = ((25 * APB1_CLOCK_FREQ) / (4 * ulBaud));
    uint32_t ulRegValue = (ulIntDiv / 100) << 4;
    uint32_t ulFlDiv = ulIntDiv - (100 * (ulRegValue >> 4));
    ulRegValue |= ((((ulFlDiv * 16) + 50) / 100)) & 0x0F;

    USART3->BRR = ulRegValue;

    IRQ_SET_PRIO(USART3_IRQn, 6, 1);
    IRQ_CLEAR(USART3_IRQn);
    IRQ_ENABLE(USART3_IRQn);

    DMA1_Channel3->CNDTR = UART3_DMA_RX_BUFFER_SIZE;
    DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;
    DMA1_Channel3->CMAR = (uint32_t)pubUART3DMABuffer;
    DMA1_Channel3->CCR = (1 << 12) | DMA_CCR3_MINC | DMA_CCR3_CIRC | DMA_CCR3_HTIE | DMA_CCR3_TCIE | DMA_CCR3_EN; // Medium priority, Memory increment, Circular mode, Half-transfer interrupt, Transfer Complete interrupt, Enable

    IRQ_SET_PRIO(DMA1_Channel3_IRQn, 6, 0);
    IRQ_CLEAR(DMA1_Channel3_IRQn);
    IRQ_ENABLE(DMA1_Channel3_IRQn);
}
void uart3_write_byte(const uint8_t ubData)
{
    while(!(USART3->SR & USART_SR_TXE));

    USART3->DR = ubData;
}
uint8_t uart3_read_byte()
{
    if(!uart3_available())
        return 0;

    uint8_t ubData;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ubData = pubUART3FIFO[usUART3FIFOReadPos++];

        if(usUART3FIFOReadPos >= UART3_FIFO_SIZE)
            usUART3FIFOReadPos = 0;
    }

    return ubData;
}
uint32_t uart3_available()
{
    return (UART3_FIFO_SIZE + usUART3FIFOWritePos - usUART3FIFOReadPos) % UART3_FIFO_SIZE;
}
void uart3_flush()
{
    usUART3FIFOReadPos = usUART3FIFOWritePos = 0;
}

static volatile uint8_t *pubUART4DMABuffer = NULL;
static volatile uint8_t *pubUART4FIFO = NULL;
static volatile uint16_t usUART4FIFOWritePos, usUART4FIFOReadPos;

void _uart4_isr()
{
    if(UART4->SR & USART_SR_IDLE)
    {
        REG_DISCARD(&UART4->SR); // Read SR & DR to clear interrupt flag
        REG_DISCARD(&UART4->DR);

        uint32_t ulSize = (UART4_DMA_RX_BUFFER_SIZE >> 1) - ((DMA2_Channel3->CNDTR - 1) % (UART4_DMA_RX_BUFFER_SIZE >> 1)) - 1;

        if(ulSize)
        {
            DMA2_Channel3->CCR &= ~DMA_CCR3_EN; // Disable DMA channel

            volatile uint8_t *pubDMABufferReadPos = (DMA2_Channel3->CNDTR > (UART4_DMA_RX_BUFFER_SIZE >> 1)) ? pubUART4DMABuffer : (pubUART4DMABuffer + (UART4_DMA_RX_BUFFER_SIZE >> 1));

            while(ulSize--)
            {
                pubUART4FIFO[usUART4FIFOWritePos++] = *pubDMABufferReadPos++;

                if(usUART4FIFOWritePos >= UART4_FIFO_SIZE)
                    usUART4FIFOWritePos = 0;
            }

            DMA2->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3; // Clear all interrupt flags otherwise DMA won't start again
            DMA2_Channel3->CNDTR = UART4_DMA_RX_BUFFER_SIZE;
            DMA2_Channel3->CMAR = (uint32_t)pubUART4DMABuffer;
            DMA2_Channel3->CCR |= DMA_CCR3_EN; // Enable DMA channel
        }
    }
}
void _dma2_channel3_isr()
{
    volatile uint8_t *pubDMABufferReadPos = 0;

    if(DMA2->ISR & DMA_ISR_HTIF3)
    {
        DMA2->IFCR = DMA_IFCR_CHTIF3; // Clear Half-transfer flag

        pubDMABufferReadPos = pubUART4DMABuffer;
    }
    else if(DMA2->ISR & DMA_ISR_TCIF3)
    {
        DMA2->IFCR = DMA_IFCR_CTCIF3; // Clear Transfer complete flag

        pubDMABufferReadPos = pubUART4DMABuffer + (UART4_DMA_RX_BUFFER_SIZE >> 1);
    }

    if(pubDMABufferReadPos)
    {
        uint32_t ulSize = (UART4_DMA_RX_BUFFER_SIZE >> 1);

        while(ulSize--)
        {
            pubUART4FIFO[usUART4FIFOWritePos++] = *pubDMABufferReadPos++;

            if(usUART4FIFOWritePos >= UART4_FIFO_SIZE)
                usUART4FIFOWritePos = 0;
        }
    }
}
void uart4_init(uint32_t ulBaud)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_UART4RST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_UART4RST;

    RCC->APB1ENR |= RCC_APB1ENR_UART4EN; // Enable USART peripheral clock

    RCC->AHBENR |= RCC_AHBENR_DMA2EN; // Enable DMA2 peripheral clock

    free((uint8_t *)pubUART4DMABuffer);
    free((uint8_t *)pubUART4FIFO);

    pubUART4DMABuffer = (volatile uint8_t *)malloc(UART4_DMA_RX_BUFFER_SIZE);

    if(!pubUART4DMABuffer)
        return;

    memset((uint8_t *)pubUART4DMABuffer, 0, UART4_DMA_RX_BUFFER_SIZE);

    pubUART4FIFO = (volatile uint8_t *)malloc(UART4_FIFO_SIZE);

    if(!pubUART4FIFO)
    {
        free((void *)pubUART4DMABuffer);

        return;
    }

    memset((uint8_t *)pubUART4FIFO, 0, UART4_FIFO_SIZE);

    usUART4FIFOWritePos = 0;
    usUART4FIFOReadPos = 0;

    UART4->CR1 = USART_CR1_UE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE;
    UART4->CR2 = 0x0000;
    UART4->CR3 = USART_CR3_DMAR;

    uint32_t ulIntDiv = ((25 * APB1_CLOCK_FREQ) / (4 * ulBaud));
    uint32_t ulRegValue = (ulIntDiv / 100) << 4;
    uint32_t ulFlDiv = ulIntDiv - (100 * (ulRegValue >> 4));
    ulRegValue |= ((((ulFlDiv * 16) + 50) / 100)) & 0x0F;

    UART4->BRR = ulRegValue;

    IRQ_SET_PRIO(UART4_IRQn, 6, 1);
    IRQ_CLEAR(UART4_IRQn);
    IRQ_ENABLE(UART4_IRQn);

    DMA2_Channel3->CNDTR = UART4_DMA_RX_BUFFER_SIZE;
    DMA2_Channel3->CPAR = (uint32_t)&UART4->DR;
    DMA2_Channel3->CMAR = (uint32_t)pubUART4DMABuffer;
    DMA2_Channel3->CCR = (1 << 12) | DMA_CCR3_MINC | DMA_CCR3_CIRC | DMA_CCR3_HTIE | DMA_CCR3_TCIE | DMA_CCR3_EN; // Medium priority, Memory increment, Circular mode, Half-transfer interrupt, Transfer Complete interrupt, Enable

    IRQ_SET_PRIO(DMA2_Channel3_IRQn, 6, 0);
    IRQ_CLEAR(DMA2_Channel3_IRQn);
    IRQ_ENABLE(DMA2_Channel3_IRQn);
}
void uart4_write_byte(const uint8_t ubData)
{
    while(!(UART4->SR & USART_SR_TXE));

    UART4->DR = ubData;
}
uint8_t uart4_read_byte()
{
    if(!uart4_available())
        return 0;

    uint8_t ubData;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ubData = pubUART4FIFO[usUART4FIFOReadPos++];

        if(usUART4FIFOReadPos >= UART4_FIFO_SIZE)
            usUART4FIFOReadPos = 0;
    }

    return ubData;
}
uint32_t uart4_available()
{
    return (UART4_FIFO_SIZE + usUART4FIFOWritePos - usUART4FIFOReadPos) % UART4_FIFO_SIZE;
}
void uart4_flush()
{
    usUART4FIFOReadPos = usUART4FIFOWritePos = 0;
}

void uart5_init(uint32_t ulBaud)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_UART5RST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_UART5RST;

    RCC->APB1ENR |= RCC_APB1ENR_UART5EN; // Enable USART peripheral clock

    UART5->CR1 = USART_CR1_UE | USART_CR1_TE;
    UART5->CR2 = 0x0000;
    UART5->CR3 = 0x0000;

    uint32_t ulIntDiv = ((25 * APB1_CLOCK_FREQ) / (4 * ulBaud));
    uint32_t ulRegValue = (ulIntDiv / 100) << 4;
    uint32_t ulFlDiv = ulIntDiv - (100 * (ulRegValue >> 4));
    ulRegValue |= ((((ulFlDiv * 16) + 50) / 100)) & 0x0F;

    UART5->BRR = ulRegValue;
}
void uart5_write_byte(const uint8_t ubData)
{
    while(!(UART5->SR & USART_SR_TXE));

    UART5->DR = ubData;
}
