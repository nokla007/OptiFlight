#include <i2c.h>
#include <ktimes.h>

#define I2C_TIMEOUT 30
/*
* @private
* private functions
*/

/*
* @brief
* Config GPIO for I2C \
* Pin SCL and SDA
*/
void _I2C_GPIO_Config(GPIO_TypeDef* gpio_scl, uint16_t pin_scl, GPIO_TypeDef* gpio_sda, uint16_t pin_sda) {
    RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB CLOCK
    // PB7 -> SDA, PB8 -> SCL
    gpio_scl->MODER |= (2 << (pin_scl * 2));    // PB7 and PB8 alternate funciton mode
    gpio_sda->MODER |= (2 << (pin_sda * 2));    // PB7 and PB8 alternate funciton mode
    gpio_scl->OTYPER |= (1 << pin_scl);     // Bit7=1, Bit8=1  output open drain
    gpio_sda->OTYPER |= (1 << pin_sda);     // Bit7=1, Bit8=1  output open drain
    gpio_scl->OSPEEDR |= (3 << (pin_scl * 2));  // High Speed for PIN PB7; High Speed for PIN PB8
    gpio_sda->OSPEEDR |= (3 << (pin_sda * 2));  // High Speed for PIN PB7; High Speed for PIN PB8
    // gpio_scl->PUPDR |= (1 << (pin_scl * 2));    // Pull up for PIN PB8; pull up for PIN PB8
    // gpio_sda->PUPDR |= (1 << (pin_sda * 2));    // Pull up for PIN PB8; pull up for PIN PB8
    if ((pin_scl >> 3 == 0)) {
        gpio_scl->AFRL |= (4 << (pin_scl * 4)); 				// PB7 AF4 (AF4 is for I2C related)
    }
    else {
        uint16_t pin = pin_scl % 8;
        gpio_scl->AFRH |= (4 << (pin * 4));
    }
    if ((pin_sda >> 3 == 0)) {
        gpio_sda->AFRL |= (4 << (pin_sda * 4)); 				// PB7 AF4 (AF4 is for I2C related)
    }
    else {
        uint16_t pin = pin_sda % 8;
        gpio_sda->AFRH |= (4 << (pin * 4));
    }
    // gpio_scl->AFRH |= (4 << 0);  				// PB8 AF4
    // gpio_sda->AFRL |= (4 << 28); 				// PB7 AF4 (AF4 is for I2C related)

}


void I2C_Start(I2C_TypeDef* i2c) {
    i2c->CR1 |= (1 << 10); // Enable the ACK
    i2c->CR1 |= (1 << 8);  // Generate START
    while (!(i2c->SR1 & (1 << 0)));  // Wait fror SB bit to set
}


void I2C_Write(I2C_TypeDef* i2c, uint8_t data) {
    /**** STEPS FOLLOWED  ************
    1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
    2. Send the DATA to the DR Register
    3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
    */
    while (!(i2c->SR1 & (1 << 7)));  // wait for TXE bit to set
    i2c->DR = data;
    while (!(i2c->SR1 & (1 << 2)));  // wait for BTF bit to set
}

bool I2C_Address(I2C_TypeDef* i2c, uint8_t address) {
    /**** STEPS FOLLOWED  ************
    1. Send the Slave Address to the DR Register
    2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
    3. clear the ADDR by reading the SR1 and SR2
    */
    uint16_t t = getTime() + I2C_TIMEOUT;
    uint16_t i = 0;
    uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
    I2C1->DR = address;
    // uint8_t addr_check = I2C1->SR1;
    while (!(I2C1->SR1 & (1 << 1))) {// wait for ADDR bit to set
        i++;
        if (getTime() > t) {
            return false;
        }
    }

    temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit

    return true;
}

void I2C_Stop(I2C_TypeDef* i2c) {
    I2C1->CR1 |= (1 << 9);  // Stop I2C

}

void I2C_WriteMulti(I2C_TypeDef* i2c, uint8_t* data, uint8_t size) {
    /**** STEPS FOLLOWED  ************
    1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
    2. Keep Sending DATA to the DR Register after performing the check if the TXE bit is set
    3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
    */
    uint16_t t = getTime() + I2C_TIMEOUT;
    while (!(I2C1->SR1 & (1 << 7)));  // wait for TXE bit to set
    while (size) {
        while (!(I2C1->SR1 & (1 << 7)));  // wait for TXE bit to set
        I2C1->DR = (uint32_t)*data++;  // send data
        size--;
    }

    while (!(I2C1->SR1 & (1 << 2))) {
        if (getTime() > t) {
            return;
        }
    }  // wait for BTF to set
}


void I2C_Read(I2C_TypeDef* i2c, uint8_t devAddr, uint8_t* buffer, uint8_t size) {
    /**** STEPS FOLLOWED  ************
    1. If only 1 BYTE needs to be Read
        a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
        b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
        c) Wait for the RXNE (Receive Buffer not Empty) bit to set
        d) Read the data from the DR

    2. If Multiple BYTES needs to be read
        a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
        b) Clear the ADDR bit by reading the SR1 and SR2 Registers
        c) Wait for the RXNE (Receive buffer not empty) bit to set
        d) Read the data from the DR
        e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
        f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the
             second last data byte (after second last RxNE event)
        g) In order to generate the Stop/Restart condition, software must set the STOP/START bit
             after reading the second last data byte (after the second last RxNE event)
    */
    uint16_t t = getTime() + I2C_TIMEOUT;
    int remaining = size;
    /**** STEP 1 ****/
    if (size == 1) {
        /**** STEP 1-a ****/
        // I2C1->DR = devAddr | 1;  //  send the address in read mode
        // while (!(I2C1->SR1 & (1 << 1)));  // wait for ADDR bit to set
        I2C_Address(i2c, devAddr | 1);
        /**** STEP 1-b ****/
        I2C1->CR1 &= ~(1 << 10);  // clear the ACK bit
        uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
        I2C1->CR1 |= (1 << 9);  // Stop I2C

        /**** STEP 1-c ****/
        while (!(I2C1->SR1 & (1 << 6)));  // wait for RxNE to set

        /**** STEP 1-d ****/
        buffer[size - remaining] = I2C1->DR;  // Read the data from the DATA REGISTER

    }

    /**** STEP 2 ****/
    else {
        /**** STEP 2-a ****/
        // I2C1->DR = devAddr | 1;  //  send the address
        // while (!(I2C1->SR1 & (1 << 1)));  // wait for ADDR bit to set
        I2C_Address(i2c, devAddr | 1);
        /**** STEP 2-b ****/
        uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit

        while (remaining > 2) {
            /**** STEP 2-c ****/
            while (!(I2C1->SR1 & (1 << 6)));  // wait for RxNE to set

            /**** STEP 2-d ****/
            buffer[size - remaining] = I2C1->DR;  // copy the data into the buffer

            /**** STEP 2-e ****/
            I2C1->CR1 |= 1 << 10;  // Set the ACK bit to Acknowledge the data received

            remaining--;
        }

        // Read the SECOND LAST BYTE
        while (!(I2C1->SR1 & (1 << 6)));  // wait for RxNE to set
        buffer[size - remaining] = I2C1->DR;

        /**** STEP 2-f ****/
        I2C1->CR1 &= ~(1 << 10);  // clear the ACK bit

        /**** STEP 2-g ****/
        I2C1->CR1 |= (1 << 9);  // Stop I2C

        remaining--;

        // Read the Last BYTE
        while (!(I2C1->SR1 & (1 << 6)));  // wait for RxNE to set
        buffer[size - remaining] = I2C1->DR;  // copy the data into the buffer
    }

}


/*
* @public
* public functions
*/

/*
* @brief only I2C1 is supported now.
*/

bool DRV_I2C_INIT(I2C_TypeDef* i2c) {
    if (i2c != I2C1) return false; // terminate if not I2C1
    // Config GPIO
    _I2C_GPIO_Config(GPIOB, 8, GPIOB, 9);
    // Enable the I2C CLOCK
    RCC->APB1ENR |= (1 << 21);  // enable I2C CLOCK
    // Reset the I2C
    I2C1->CR1 |= (1 << 15);
    I2C1->CR1 &= ~(1 << 15);
    // Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
    I2C1->CR2 |= (45 << 0);  // PCLK1 FREQUENCY in MHz
    // Configure the clock control registers
    // I2C1->CCR = 225 << 0;
    I2C1->CCR = 0x26 << 0; //according to cubemx setup
    I2C1->CCR |= (1 << 15); //master mode selection
    // Configure the rise time register
    // I2C1->TRISE = 46;
    I2C1->TRISE = 14;   // according to cubemx
    // Program the I2C_CR1 register to enable the peripheral
    I2C1->CR1 |= (1 << 0);  // Enable I2C
    return true;
}

/********
 * @brief
 * Write a byte to a device register
*/
bool _I2C_MEM_WRITE(I2C_TypeDef* i2c, uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    I2C_Start(i2c);
    bool check = I2C_Address(i2c, devAddr);
    if (check == false) return false;
    delay_ms(10);
    I2C_Write(i2c, regAddr);
    I2C_Write(i2c, data);
    I2C_Stop(i2c);
    return true;
}

/********
 * @brief
 * Read bytes from a device register
*/
bool _I2C_MEM_READ(I2C_TypeDef* i2c, uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint16_t size) {
    I2C_Start(i2c);
    bool check = I2C_Address(i2c, devAddr);
    if (check == false) return false;
    delay_ms(10);
    I2C_Write(i2c, regAddr);
    delay_ms(10);
    I2C_Start(i2c);
    I2C_Read(i2c, devAddr, buffer, size);
    I2C_Stop(i2c);
    return true;
}