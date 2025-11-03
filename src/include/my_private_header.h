// ================== my_private_header.h ==================
// Project: Smart Irrigation System – Final Project
// Target: C8051F380 Microcontroller
// Overview:
// Low-level definitions and control functions:
// ----------------------------------------------------------
// [1] Include Compiler and MCU Definitions:
//     -> compiler_defs.h, C8051F380_defs.h
// [2] Constants and Timings:
//     -> I²C half-period timing definition (`usec`)
// [3] Pin Definitions:
//     -> Assign symbolic names for MCU pins:
//        • I²C communication pins (SDA, SCL)
//        • Relay control pin (Relay)
// [4] I²C Communication Functions:
//     -> Low-level bit-banging implementation:
//        • START/STOP conditions
//        • Byte write/read functions
// [5] LM75 Temperature Sensor Function:
//     -> Temperature sensor reading function (`readTemp`)
// [6] DS1307 RTC Control Functions:
//     -> Read/Write RTC registers, time setup, printing
//     -> BCD <-> Decimal conversion functions for RTC data formatting
// [7] ADC Conversion Function:
//     -> Read ADC channel values (soil, rain, light sensors)
// [8] Servo PWM Control Function:
//     -> PWM signal generation via PCA module
// [9] Relay Control Functions:
//     -> Relay activation/deactivation (pump control)
#include "compiler_defs.h"       // Include compiler definitions (macros, typedefs, etc.)  
#include "C8051F380_defs.h"      // Include SFR definitions for the C8051F380  
#define usec 3   // Half-period delay in microseconds for manual I²C bit-banging
                 // - Controls the HIGH/LOW time for SCL when toggling via GPIO
                 // - Total clock period = 2 × usec -> 6 µs here ˜ 166 kHz I²C speed
                 // - Timing set manually (software-driven), not using built-in SMBus/I²C hardware
 
// ---------- I2C Pin Definitions ----------  
sbit SDA = P1^1;                // I2C data line (SDA) connected to Port 1, Pin 1  
sbit SCL = P1^0;                // I2C clock line (SCL) connected to Port 1, Pin 0  

// ---------- Relay Pin Definition ----------  
sbit Relay = P0^2;              // Relay control pin (active-high) on Port 0, Pin 2  

// ======================= I²C FUNCTIONS (Bit-Banged) ======================= Inter-Integrated Circuit
// I²C Protocol Sequence (master):
// Step 1: START condition
// Step 2: [Slave Address + R/W bit] (MSB first)
// Step 3: ACK from slave (SDA pulled LOW by slave on 9th clock)
// Step 4: [Data Byte(s)] (MSB first; master clocks each bit)
// Step 5: ACK/NACK per byte (master releases SDA for slave ACK when writing,
// or master drives ACK/NACK after reads)
// Final:  STOP condition
// Implementation notes (this module):
// - Manual I²C using bit-banging (software-driven timing), not the built-in SMBus/I²C HW.
// - Lines used in this project: SCL = P1.0 (Push-Pull), SDA = P1.1 (Open-Drain + Pull-Up).
// - Bus speed is set by delays: SCL period ˜ 2 * usec (in microseconds).
//   Example: usec = 3 -> T_SCL ˜ 6 µs -> f_SCL ˜ 166.7 kHz.
// - START = SDA falling while SCL is HIGH; STOP = SDA rising while SCL is HIGH.
// - This implementation assumes master-driven SCL (no clock stretching by slaves).
// ==========================================================================
// Step 1: START condition
void startI2c(void) // Step 1: START condition
{
    SDA = 1;               // Ensure SDA is released high (idle)
    SCL = 1;               // Ensure SCL is released high (bus idle)
    delay_us(usec);        // Stabilization delay (defines timing; see usec)
    SDA = 0;               // SDA goes LOW while SCL is HIGH -> START condition
    delay_us(usec);        // Allow slaves to detect START
    SCL = 0;               // Pull SCL LOW to begin the data/clock phase
}
/*
 * writeByteI2c(): Sends a single byte over I²C, MSB first, and receives ACK/NACK.
 * Step 2: [Address + R/W] or data byte transmission
 * Step 3: Wait for ACK from slave (ACK = SDA pulled LOW)
 * Process:
 *   - Sends 8 bits, one at a time (MSB first) using SDA and toggling SCL.
 *   - Releases SDA for 9th bit to allow slave to respond with ACK or NACK.
 * Returns:
 *   - 0 = ACK received (slave pulled SDA low)
 *   - 1 = NACK (slave left SDA high)
 * Notes:
 *   - Timing is software-defined via delay_us(usec) on each edge (bit-banging).
 *   - Ensure 'startI2c()' was called before the first write, and 'stopI2c()' after the last byte.
 */
bit writeByteI2c(U8 outchar) // Step 2 + Step 3: Send byte + receive ACK
{
    bit ack;
    U8 i;
    for(i = 0; i < 8; i++)
    {
        outchar = outchar << 1;  // Shift MSB into CY (carry)
        SDA = CY;                // Output current bit on SDA (MSB first)
        delay_us(usec);          // Setup time before clock HIGH
        SCL = 1;                 // Clock HIGH -> slave samples SDA
        delay_us(usec);          
        SCL = 0;                 // Clock LOW -> prepare next bit
    }

    delay_us(usec);              // Wait before ACK cycle

    SDA = 1;                     // Release SDA for slave to send ACK on 9th clock
    SCL = 1;                     // Clock HIGH to sample ACK
    delay_us(usec);              
    ack = SDA;                   // Read SDA: 0 = ACK, 1 = NACK
    SCL = 0;                     // Clock LOW to complete ACK cycle
    delay_us(usec);              
    return ack;                  // Return ACK/NACK status
}
/*
 * readByteI2c(): Reads a single byte from the I²C bus.
 * Step 4: Receive data byte from slave
 * Step 5: Send ACK (0) to continue or NACK (1) to end transmission
 * Process:
 *   - Reads 8 bits (MSB first) by releasing SDA and clocking in bits from slave.
 *   - After 8 bits, master responds with ACK (ask_master = 0) or NACK (ask_master = 1).
 * Parameter:
 *   ask_master = 0 to send ACK (continue reading), 1 to send NACK (end transmission)
 * Returns:
 *   Byte received from slave.
 * Notes:
 *   - Use ACK (0) for all bytes except the last; send NACK (1) on the final byte before STOP.
 *   - Timing is software-defined via delay_us(usec) on each edge (bit-banging).
 */
U8 readByteI2c(bit ask_master) // Step 4 + Step 5: Read byte and send ACK/NACK
{
    U8 i2cData = 0;
    U8 i;
    SDA = 1;                    // Release SDA -> input mode (slave drives SDA)
    for(i = 7; ; i--)
    {
        SCL = 1;                // Clock HIGH -> slave outputs current bit on SDA
        delay_us(usec);         
        if(SDA)                 // If SDA is HIGH, set corresponding bit
            i2cData |= (1 << i);
        SCL = 0;                // Clock LOW -> prepare for next bit
        delay_us(usec);         
        if(i == 0) break;       // Exit after 8 bits
    }
    SDA = ask_master;          // Master drives ACK (0) to continue, or NACK (1) to stop
    delay_us(usec);            
    SCL = 1;                    // Clock HIGH to send ACK/NACK on the 9th clock
    delay_us(usec);            
    SCL = 0;                    // Clock LOW to complete ACK/NACK cycle
    delay_us(usec);            
    return i2cData;             // Return received byte
}
/*
 * stopI2c(): Generates an I²C STOP condition manually.
 * Step 5: STOP bit
 *   - SDA goes from LOW to HIGH while SCL is HIGH
 *   - Signals end of communication on the bus
 * Notes:
 *   - Call STOP only after all bytes have been written/read (and after a final NACK on reads).
 *   - In repeated-transfer scenarios, you may issue a repeated START instead of STOP.
 */
void stopI2c(void) // Step 5: STOP condition
{
    SDA = 0;               // Hold SDA low before STOP
    SCL = 1;               // Raise SCL to prepare for STOP condition
    delay_us(usec);        // Setup delay (defines timing; see usec)
    SDA = 1;               // SDA goes HIGH while SCL is HIGH -> STOP condition
    delay_us(usec);        // Ensure STOP is registered by all slaves
}
// ---------- LM75 TEMPERATURE SENSOR Function ----------
/*
 * readTemp(): Reads and decodes the temperature value from the LM75 sensor over I²C.
 * Description:
 *   - LM75 outputs temperature as a 9-bit value spread across 2 bytes: MSB and LSB.
 *   - MSB (Byte 1): Bits [7:0] contain the upper 8 bits of temperature data.
 *   - LSB (Byte 2): Only bit 7 is used (bit 8 of temperature); bits [6:0] are unused.
 *   - The combined 16-bit word is right-shifted by 5 to extract the 9 significant bits.
 *   - Each LSB equals 0.125°C (resolution), so final value = raw × 0.125.
 * Bit layout (as used in this firmware):
 *   [MSB: b15 b14 b13 b12 b11 b10 b9  b8] + [LSB: b7 (used), b6..b0 (ignored)]
 *   After (MSB<<8 | LSB) >> 5 -> the 9 significant bits are aligned to LSB.
 * Worked example (hex -> bits -> °C):
 *   Suppose the device returns: MSB = 0x19 (0001 1001), LSB = 0x80 (1000 0000).
 *   1) Combine to 16-bit: (0x19 << 8) + 0x80 = 0x1980.
 *   2) Align data: 0x1980 >> 5 = 0x00CC (decimal 204).
 *   3) Convert to °C with resolution: temperature = 204 × 0.125 = 25.5 °C.
 * I²C Transaction Sequence:
 *   1) START
 *   2) Write LM75 address + Write mode (0)
 *   3) Write register address (0x00 for temperature)
 *   4) REPEATED START
 *   5) Write LM75 address + Read mode (1)
 *   6) Read MSB (ACK)
 *   7) Read LSB (NACK)
 *   8) STOP
 *
 * Parameters:
 *   add – LM75 I²C address with R/W bit = 1 (read mode).
 * Returns:
 *   float – Temperature in degrees Celsius (°C)
 */
float readTemp(U8 add)
{
    float temp;                             // Variable to store the calculated temperature
    startI2c();                             // Send I2C START condition
    if(!writeByteI2c(add))                  // Send LM75 I²C address (read mode), wait for ACK
        temp = (((readByteI2c(0) << 8)      // Read MSB (first byte) and shift to upper bits Most Significant Bit/Byte 
               + readByteI2c(1))            // Read LSB (second byte, only bit 7 is relevant) Least Significant Bit/Byte
               >> 5) * 0.125;               // Extract 9-bit value (right-shift by 5), then multiply by resolution (0.125°C/LSB)
    stopI2c();                              // Send I2C STOP condition
    return temp;                            // Return temperature as float
}

// ---------- DS1307 RTC FUNCTIONS (logical read pipeline order) ----------
// [Step index guide]
//   Step 1: Point DS1307 internal register (write phase)
//   Step 2: Read byte(s) from DS1307 (read phase)
//   Step 3: Convert BCD -> decimal
//   Step 4: Print / use values
// (Setup/writes are separate: "Init/Write" steps)

// ---- Forward declarations so we can place conversion after read ----
U8 bcdToDec(U8 val);               // Prototype for BCD->DEC conversion
U8 decToBcd(U8 val);               // Prototype for DEC->BCD conversion

// --------------------------------------------------------------------
// [Init/Write] writeDS1307(): write one DS1307 register (decimal in)
// I²C: START -> [0xD0 W] -> [reg] -> [data(BCD)] -> STOP
bit writeDS1307(U8 addr, U8 value)
{
    bit ack = 1;                                   // Track NACK occurrence (1 = NACK seen)
    startI2c();                                    // START condition
    if (!writeByteI2c(0xD0))                       // Send address+W (0xD0), check ACK (0=ACK)
    {
        ack = writeByteI2c(addr);                  // Send target register address
        ack = writeByteI2c(decToBcd(value));       // Send data (converted to BCD)
    }
    stopI2c();                                     // STOP condition
    return ack;                                    // Return 0 if full ACK path, else 1
}

// --------------------------------------------------------------------
// [Step 1+2] readDS1307(): read one register then return DECIMAL
// I²C: START->[0xD0 W]->[reg]->STOP->START->[0xD1 R]->read+NACK->STOP
U8 readDS1307(U8 addr)
{
    U8 dataVal = 0;                                // Raw BCD storage
    startI2c();                                    // START condition
    if (!writeByteI2c(0xD0))                       // Address+W (ACK expected)
    {
        writeByteI2c(addr);                        // Send register pointer (0x00..0x07)
        stopI2c();                                 // STOP to latch internal pointer
        startI2c();                                // START again (read phase)
        writeByteI2c(0xD1);                        // Address+R (0xD1)
        dataVal = readByteI2c(1);                  // Read one byte; send NACK (last byte)
    }
    stopI2c();                                     // STOP condition
    if (addr == 0) dataVal &= 0x7F;                // Clear CH bit if reading seconds
    return bcdToDec(dataVal);                      // [Step 3] Convert BCD->DEC and return
}

// --------------------------------------------------------------------
// [Init/Write] setupTime(): write HH:MM:SS (decimal inputs)
void setupTime(U8 hour, U8 minute, U8 second)
{
    writeDS1307(0x02, hour);                       // Write hours register (0x02)
    writeDS1307(0x01, minute);                     // Write minutes register (0x01)
    writeDS1307(0x00, second);                     // Write seconds register (0x00)
}

// --------------------------------------------------------------------
// [Step 4] printTime(): print HH:MM:SS from decimal fields
void printTime(U8 hour, U8 minute, U8 second)
{
    printf("%d%d:%d%d:%d%d",                      // Print HH:MM:SS with manual digits
           (int)hour / 10,   (int)hour % 10,      // Hour tens, hour units
           (int)minute / 10, (int)minute % 10,    // Minute tens, minute units
           (int)second / 10, (int)second % 10);   // Second tens, second units
}

// [Step 3 helpers] bcdToDec(): convert 8-bit BCD to decimal (0..99)
U8 bcdToDec(U8 val)
{
    return ((val >> 4) * 10)                       // Upper nibble -> tens
         + (val & 0x0F);                           // Lower nibble -> units
}
// [Init/Write helper] decToBcd(): convert decimal (0..99) to 8-bit BCD
U8 decToBcd(U8 val)
{
    return ((val / 10) << 4)                       // Tens into upper nibble
         | (val % 10);                              // Units into lower nibble
}

// ---------- ADC FUNCTION ---------- Analog-to-Digital Converter
/*
 * ADC_IN_CHANNEL(): Performs an analog-to-digital conversion on the selected ADC input channel.
 * * Parameters:
 *   channelSelect – ADC channel number (e.g., 0x00 = P2.0, 0x01 = P2.1, etc.)
 * Process:
 *   - Sets AMX0P to select the desired analog input channel.
 *   - Waits briefly (delay_us) to allow the input voltage to stabilize.
 *   - Starts the ADC conversion by setting AD0BUSY.
 *   - Waits until AD0INT is set, indicating conversion complete.
 *   - Clears the AD0INT flag.
 * Returns:
 *   - 10-bit ADC result (0 to 1023) from the selected analog input.
 */
int ADC_IN_CHANNEL(U8 channelSelect)
{
    AMX0P = channelSelect;    // Select the ADC input channel (via analog multiplexer)
    delay_us(usec);           // Short delay to allow input voltage to settle
    AD0BUSY = 1;              // Initiate ADC conversion
    while(!AD0INT);           // Wait until conversion is finished (AD0INT = 1)
    AD0INT = 0;               // Clear conversion complete flag
    return ADC0;              // Return 10-bit result from ADC0 register
}
// ---------- Servo PWM Function ----------Pulse Width Modulation
// pulse(): Generates a precise PWM signal using PCA Module 0 to control servo angle.
// OVERVIEW:
// ----------------------------------------------------
// - The PCA (Programmable Counter Array) operates at 4 MHz 
//   (derived from SYSCLK ÷ 12 = 48 MHz ÷ 12).
//   -> Each PCA tick = 0.25 microseconds.
// - To create a pulse of `w` microseconds (µs):
//   -> Convert pulse width to ticks by multiplying by 4 (since 1 µs = 4 ticks),
//     then apply 2's complement (negate the result) for proper compare logic.
//  -> Formula: width_ticks = -4 × w
//   • Example:
//       w = 1500 µs
//       -> width_ticks = -4 × 1500 = -6000
//       -> 2’s complement of -6000 = 0xE890 (decimal: 59536)
// WHY NEGATIVE?
// - The PCA counter always counts **upward** from 0 to 65535.
// - We load a **two’s complement negative value** (e.g., -6000 = 0xE890).
// - When the PCA counter reaches this value (e.g., 59536), a **compare match** occurs.
//  -> PCA sets output LOW at that exact tick -> pulse ends with precision.
// PHYSICAL BEHAVIOR:
// ----------------------------------------------------
// - Output pin goes HIGH at start of pulse.
// - PCA counter starts counting from 0.
// - When match value is reached -> pin goes LOW.
// - The remainder of the 20ms cycle stays LOW.
// PULSE WIDTH -> PCA VALUE (TICK) CONVERSION:
// ----------------------------------------------------
// Each value below passes through 3 stages:
//   1. µs width (input)
//   2. ×(-4) -> tick count (signed int16)
//   3. 2's complement -> HEX -> final value loaded into PCA
// •  600 µs -> -2400   -> 2's Comp = 63168  -> HEX = 0xF6C0  -> 0°
// • 1500 µs -> -6000   -> 2's Comp = 59536  -> HEX = 0xE890  -> 90°
// • 2400 µs -> -9600   -> 2's Comp = 55936  -> HEX = 0xDA80  -> 180°
// - In each step, we increase or decrease the angle by 30 µs ( = 120 ticks ).
//    -> Total movement from 600 to 2400 = 1800 µs -> 1800 / 30 = 60 steps
//    -> Full sweep (up + down) = 120 steps total
// - Each pulse is sent every 20ms (delay_ms(20))
//    -> Full sweep takes 120 × 20ms = 2400ms 
// - The 30µs step size was chosen to:
///    -> Ensure smooth motion
//     ->Avoid delay accumulation
//     -> Allow precise, visible servo motion without jitter
// REGISTERS USED:
// ----------------------------------------------------
// - PCA0CPL0 -> Capture Low Byte  (bits 0–7 of compare value)
// - PCA0CPH0 -> Capture High Byte (bits 8–15)
// - Together form a full 16-bit match register.
// REMARK:
// ----------------------------------------------------
// - `width` is clamped between 600–2400 µs to avoid unstable angles.
// - No interrupt is needed — PCA handles the match and pin toggle automatically.
void pulse(U16 width)
{
    if (width < 600) width = 600;         // Limit minimum pulse width to 600 µs (0°)
                                          // -> Protection: Prevents sending too narrow pulse, which could cause servo jitter or fail to respond
    else if (width > 2400) width = 2400;  // Limit maximum pulse width to 2400 µs (180°)
                                          // -> Protection: Prevents over-driving the servo beyond safe range, which may damage internal gears
    width = -4 * width;                   // Convert microseconds to PCA ticks (negative for compare match logic) 
    PCA0CPL0 = width & 0xFF;              // Load lower 8 bits of match value to PCA0CPL0   (bits 0–7 of compare value)
                                          // Example: for width = 1500 -> -6000 -> 0xE890 -> LSB = 0x90        
    PCA0CPH0 = (width >> 8);              // Load upper 8 bits to PCA0CPH0  Capture High Byte (bits 8–15)
                                          // Example: MSB = 0xE8 for 1500 µs pulse 	// the pca convert them to one adress of 16 
	                                        //  bits to compare word again  Together form a full 16-bit match register.
}

// ---------- Relay Control Functions ----------
// This module controls a 5V relay (low-side switching via NPN transistor).
// MCU pin P0.2 sends a 3.3V logic signal to the relay module's IN pin.
// HIGH -> transistor saturates -> coil energized -> NO contact closes -> pump ON.
// LOW  -> transistor off -> coil de-energized -> contact opens -> pump OFF.
// The module includes a flyback diode to protect the transistor from coil back-EMF.

// Turns relay ON (P0.2 HIGH -> transistor ON -> coil energized -> pump ON)
void Relay_On(void)
{
    Relay = 1;    // Drive P0.2 high -> bias transistor -> close coil circuit -> pump runs
}

// Turns relay OFF (P0.2 LOW -> transistor OFF -> coil de-energized -> pump OFF)
void Relay_Off(void)
{
    Relay = 0;    // Drive P0.2 low -> cut transistor -> open coil circuit -> pump stops
}


