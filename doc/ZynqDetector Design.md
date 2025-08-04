# ZynqDetector Design

This document describes design of the FreeRTOS program runs in Zynq-based detectors. It includes the design of both base class zynqDetector and derived classes for specific detectors (GeRM, PYNQ-SSD, etc).

## 1. Base Class zynqDetector

### 1.1 Architecture

The program processes requests from EPICS IOC through UDP datagrams, read/write the registers, and send the readout data back to IOC through UDP datagrams. The architecture is depicted in Fig.1.

![](zynq-detector-data-flow.png)

When the program starts, it creates the following tasks:

- UDP receive task. This task receives UDP datagrams from EPICS IOC and dispatch access requests to other tasks.
- Fast access task. This task processes fast access requests that are direct register read/write operations.
- Slow access task. This task processes slow access requests that read/write slow devices (I2C, SPI, etc), if any.
- UDP transmit task. This task sends responses containing readout data to EPICS IOC.

The requests and responses are delivered among the tasks using FreeRTOS queues.

### 1.2 Periodical Reporting

The following information is reported to the IOC without being requested:

- Device status

### 1.3 Functions

- Initialization
  - network_init();
    - void read_network_config();
    - bool string_to_addr();
  - virtual void interrupt_init();
  - virtual void queue_init();
  - virtual void task_init()
    - udp_rx_task()
    - udp_tx_task()
    - 

- Data processing
  - rx_msg_proc();
  - tx_msg_proc();

## 2. Derived Classes

### 2.1 GeRM

#### 2.1.1 Registers

GeRM access the following registers

Register | Read/Write | Functions | Note
:-:|:-:|:-:|:-
EVENT_FIFO_CNTRL | Read, write | `fifo_enable()`<br>`fifo_disable()`<br>`fifo_reset()` |
DETECTOR_TYPE | Write | `zDDM_init()` |  Detector configuration.
MARS_RDOUT_ENB | Write | `zDDM_init()` | Detector configuration.
TRIG | Read, Write | `zDDM_arm()`<br>`zDDM_reset()`<br>`check_framestatus()` | 
FRAME_NO | Read | `get_framenum()`<br>`zDDM_arm()`<br>`special()` |
COUNT_TIME_LO<br>COUNT_TIME_HI | Read, Write | `zDDM_write_preset()`<br>`get_framenum()`
MARS_CONF_LOAD | Write | `latch_conf()`<br>`stuff_mars()` | 
ADC_SPI | Write | `send_spi_bit()`<br>`ad9252_cnfg()` | 
EVENT_FIFO_CNT | Read | `fifo_numwords()` |
EVENT_FIFO_DATA | Read | `fifo_getdata()` |
VERSIONREG | Read | `init_record()` |
MARS_PIPE_DELAY | Write | `init_record()`<br>`special()` |
TD_CAL | Write | `init_record()`<br>`special()` | 
COUNT_MODE | Write | `special()` |
CALPULSE_RATE | Write | `special()` |
CALPULSE_WIDTH | Write | `special()` |
CALPULSE_CNT | Write | `special()` |
MARS_CALPULSE | Write | `special()` |
CALPULSE_MODE | Write | `special()` |
UDP_IP_ADDR | Write | `special()` |

The access methods are categorized into three modes.

##### 2.1.1.2 Register Direct Access

This mode accesses a single register address. A typical application is writing a register.

##### 2.1.1.2 Register Direct Access with Hold

This mode accesses a single register address and hold for response. A typical application is reading a register.

##### 2.1.1.3 Register Group Access

This mode accesses a group of register addresses. A typical application is continuous writing one or more registers.

##### 2.1.1.4 Register Group Access with Hold

This mode accesses a group of register addresses, during which it may wait for response. A typical application is continuous writing one or more registers.

#### 2.1.2 IOC Functions

- `init_record()`

Original:

```c++
pl_register_read(fd,VERSIONREG); // without no hold
pl_register_write(fd,MARS_PIPE_DELAY,pscal->pldel); // write 72
pl_register_write(fd,TD_CAL,pscal->rodel); // write 15
```

Suggested chanages:

```c++
single_reg_access( VERSIONREG );
single_reg_write( MARS_PIPE_DELAY, ps->pldel );
single_reg_write( TD_CAL, ps->rodel );
```

- `process()`

Original:

```c++
pl_register_read(fd,FRAME_NO); // Wait for response in the original code. Maybe without waiting?
pl_register_read(fd,FRAME_NO); // Wait for response in the original code. Maybe without waiting?
```

Suggested chanages:

```c++
single_reg_read( FRAME_NO );
```

- `special()`

Original:

```c++
pl_register_write(fd,FRAME_NO,pscal->runno);  // no wait
pl_register_write(fd,MARS_PIPE_DELAY, pscal->pldel);  // no wait
pl_register_write(fd,TD_CAL,pscal->rodel);  // no wait
pl_register_write(fd,COUNT_MODE,0);  // no wait

pl_register_write(fd,CALPULSE_RATE,(25000000/pscal->tpfrq/2));  // (likely) no wait
pl_register_write(fd,CALPULSE_WIDTH,(25000000/pscal->tpfrq/2));  // (likely) no wait
pl_register_write(fd,CALPULSE_CNT,pscal->tpcnt); // (likely) no wait

pl_register_write(fd,MARS_CALPULSE,0xFFF); // (likely) no wait
pl_register_write(fd,CALPULSE_MODE,1); // (likely) no wait

pl_register_write(fd,UDP_IP_ADDR,addr);  // no wait
```

Suggested chanages:

```c++
single_reg_write( FRAME_NO, pscal->runno );
single_reg_write( MARS_PIPE_DELAY, pscal->pldel );
single_reg_write( TD_CAL,pscal->rodel );
single_reg_write( CALPULSE_RATE,(25000000/pscal->tpfrq/2) );
single_reg_write( CALPULSE_WIDTH,(25000000/pscal->tpfrq/2) );
single_reg_write( CALPULSE_CNT,pscal->tpcnt );
single_reg_write( MARS_CALPULSE,0xFFF );
single_reg_write( CALPULSE_MODE,1 );
single_reg_write( UDP_IP_ADDR,addr );

```

- `fifo_enable()` / `fifo_disable()`

Original:

This function may no longer be needed if adopting UDP streaming for data acquisition.

```c++
// toggle bit 2 of EVENT_FIFO_CNTRL twice
pl_register_read(fd,EVENT_FIFO_CNTRL); // must wait for response before proceeding
pl_register_write(fd,EVENT_FIFO_CNTRL,newval); // no wait
```

Suggested chanages:

```c++
single_reg_read_block( EVENT_FIFO_CNTRL );
single_reg_write( EVENT_FIFO_CNTRL, newval );
```

- `check_framestatus()`

Original:

```c++
pl_register_read(fd, TRIG); // no wait
```

Suggested chanages:

```c++
single_reg_read(  );
```

- `get_framenum()`

Original:

```c++
pl_register_read(fd, FRAME_NO); // no wait
```

Suggested chanages:

```c++
single_reg_read( FRAME_NO );
```

- `get_framelen()`

Original:

```c++
pl_register_read(fd, COUNT_TIME_LO);
pl_register_read(fd, COUNT_TIME_HI); // wait for response to calculate count_time. maybe without waiting?
```

Suggested chanages:

```c++
single_reg_read( COUNT_TIME_LO );
single_reg_read( COUNT_TIME_HI );
```

- `fifo_numwords()`

Original:

```c++
pl_register_read(fd, EVENT_FIFO_CNT); // no wait
```


Suggested chanages:

```c++
single_reg_read( EVENT_FIFO_CNT );
```



- `fifo_getdata(int len, int *data)`

May be removed if using UDP streaming for data acquistion.

Original:

```c++
loop for i:
    data[i] = pl_register_read(fd, EVENT_FIFO_DATA);  // wait for response.
```

Suggested chanages:

** Remove. **

- `zDDM_init(int after)`

Original:

```c++
pl_register_write(fd, DETECTOR_TYPE, 0); // no wait
pl_register_write(fd, MARS_RDOUT_ENB, 0x8aaa); // no wait
```

Suggested chanages:

```c++
single_reg_write( DETECTOR_TYPE, 0 );
single_reg_write( MARS_RDOUT_ENB, 0x8aaa );
```

- `zDDM_arm(struct zDDMRecord *pscal, int val)`

Original:

```c++
fifo_disable();/* disable fifo */
fifo_reset();
pl_register_read(fd,FRAME_NO); // no wait
fifo_enable();
pl_register_write(fd,TRIG,val); // no wait
```


Suggested chanages:

```c++
single_reg_read( FRAME_NO );
single_reg_write( TRIG, val );
```

- `zDDM_reset(struct zDDMRecord *pscal)`

Original:

```c++
fifo_disable();
pl_register_write(fd,TRIG,0); // no wait
```


Suggested chanages:

```c++
single_reg_write( TRIG, 0 );
```

- `zDDM_write_preset(zDDMRecord *psr)`

Original:

```c++
pl_register_write(fd,COUNT_TIME_LO, (uint32_t)(pr1 & 0xffffffff)); // no wait
pl_register_write(fd,COUNT_TIME_HI, (uint32_t)(pr1 >> 32)); // wait
```


Suggested chanages:

```c++
single_reg_write( COUNT_TIME_LO, (uint32_t)(pr1 & 0xffffffff) );
single_reg_write( COUNT_TIME_HI, (uint32_t)(pr1 >> 32) );
```

- `latch_conf(void)`

Original:

```c++
pl_register_write(fd,MARS_CONF_LOAD,2); // no wait
pl_register_write(fd,MARS_CONF_LOAD,0); // no wait
```


Suggested chanages:

```c++
single_reg_write( MARS_CONF_LOAD,2 );
single_reg_write( MARS_CONF_LOAD,0 );
```


- `stuff_mars(void *pscal)`

Original:

```c++
for i in 0-11:
    pl_register_write(fd,MARS_CONF_LOAD,4); // no wait
    pl_register_write(fd,MARS_CONF_LOAD,0); // no wait
    for j in 0-13:
        pl_register_write(fd,MARS_CONFIG,loads[i][j]); // no wait
        latch_conf();
    pl_register_write(fd,MARS_CONF_LOAD,0x00010000<<i); // no wait
    pl_register_write(fd,MARS_CONF_LOAD,0); // no wait
```


Suggested chanages:

```c++
for i in 0-11:
    single_reg_write( MARS_CONF_LOAD, 4); // no wait
    single_reg_write( MARS_CONF_LOAD, 0); // no wait
    for j in 0-13:
        single_reg_write( MARS_CONFIG, loads[i][j]); // no wait
        latch_conf();
    single_reg_write( MARS_CONF_LOAD, 0x00010000<<i); // no wait
    single_reg_write( MARS_CONF_LOAD, 0); // no wait

// or?
write_loads( loads ); // implement the register access in FreeRTOS program in detector
```



- `send_spi_bit(int chipSel, int val)`

Original:

```c++
sda = val & 0x1;
pl_register_write(fd,ADC_SPI,(chipSel | 0)); // no wait
pl_register_write(fd,ADC_SPI,(chipSel | sda)); // no wait
pl_register_write(fd,ADC_SPI,(chipSel | 0x2 | sda)); // no wait
pl_register_write(fd,ADC_SPI,(chipSel | sda)); // no wait
pl_register_write(fd,ADC_SPI,(chipSel | 0)); // no wait
```


Suggested chanages:

```c++
sda = val & 0x1;
single_reg_write( ADC_SPI,(chipSel | 0)); // no wait
single_reg_write( ADC_SPI,(chipSel | sda)); // no wait
single_reg_write( ADC_SPI,(chipSel | 0x2 | sda)); // no wait
single_reg_write( ADC_SPI,(chipSel | sda)); // no wait
single_reg_write( ADC_SPI,(chipSel | 0)); // no wait
```



- `load_ad9252reg(int chipSel, int addr, int data)`

Original:

```c++
send_spi_bit(chipSel,0); 
for (i=1;i>=0;i--)
    send_spi_bit(chipSel,0); // no wait
for (j=12;j>=0;j--)
    send_spi_bit(chipSel,addr >> j); // no wait
for (j=7;j>=0;j--)
    send_spi_bit(chipSel,data >> j); // no wait
```

Suggested chanages:

```c++
// no changes
```


- `ad9252_cnfg(int chipNum, int addr, int data)`

Original:

```c++
pl_register_write(fd,ADC_SPI,chipSel); // no wait
load_ad9252reg(chipSel,addr,data);
pl_register_write(fd,ADC_SPI,0b11100); // no wait
```

Suggested chanages:

```c++
single_reg_write( ADC_SPI,chipSel );
load_ad9252reg(chipSel,addr,data);
single_reg_write( ADC_SPI,0b11100 );
```

- `long zDDM_init(int after)`

Original:

```c++
ad9252_cnfg(1,22,9); /* clock skew adjust */
ad9252_cnfg(1,255,1); /* latch regs */
ad9252_cnfg(2,22,8); /* clock skew adjust */
ad9252_cnfg(2,255,1); /* latch regs */
ad9252_cnfg(3,22,8); /* clock skew adjust */
ad9252_cnfg(3,255,1); /* latch regs */
```

Suggested chanages:

```c++
// no changes
```

Function calls that cause massive register access:

- `ad9252_cnfg()` --> `load_ad9252reg()` --> `send_spi_bit()`

- `stuff_mars()` --> `latch_conf()`


## xx. Error handling

When an error occurs in an operation, the caller calls report_err() to:
- Print error log;
- Write failure number to register.

## Germanium Detector

The class GeDetector derives from ZynqDetector and is designed for the Germanium detectors.

### PS I2C interface

#### Interfaces

- GeRM
  - fe_i2c_sda @ J2-K49  SEAF8-20-10
  - fe_i2c_scl @ J2-K50  SEAF8-20-10
  - dbg0 @ J5-1
  - dbg1 @ J5-2

- MARS 384 Channel Detector Board (temperatures)
  - i2c_sda @ J1-K49 SEAM8-20-10
  - i2c_scl @ J1-K50 SEAM8-20-10

- MAIA - Ancillary Board
  - i2c_scl @ J4-17
  - i2c_scl @ J4-19

#### PVs

- Temperatures:
  - System 1: `:Temp1`
  - System 2: `:Temp2`
  - System 3: `:Temp3`
  - CPU: `:ztmp`

- HV Bias
  - Voltage
    - Setpoint: `:HV`
    - Readback: `:HV_RBV`
  - Current
    - Readback: `:HV_CUR`
  
#### ADCs/DACs

- MAIA - Ancillary Board

  - DAC
    - U14 DAC7678
      - Addr: `2b'10` (PIN) --> `0x1A` (ADDR)
      - Channels
        - A - hermes_vl0
        - B - hermes_vl1
        - C - hermes_vh1
        - D - hermes_vl2
        - E - hermes_vh2
        - F - bias_hv_set
        - G - pelt1_iset
        - H - pelt2_iset
  - ADC
    - U15 LTC2309
      - Addr: `2'b00` (PIN) --> `0x08` (ADDR)
      - Channels
        - 0: vmon_temp1
        - 1: vmon_temp2
        - 2: vmon_temp3
        - 3: vmon_temp4
        - 4: bias_vmon
        - 5: bias_imon
        - 6: pelt1_imon
        - 7: pelt2_imon

    - U16 LTC2309
      - Addr: `2'b01` (PIN) --> `0x0A` (ADDR)
      - Channels
        - 0: detileak_mon
        - 1: pelt_volt

- MARS 384 Channel Detector Board
  - Temperature sensors
    - U? TMP100
      - Addr: `00` (PIN) --> `0x48` (ADDR)
    - U? TMP100
      - Addr: `0x` (PIN) --> `0x49` (ADDR)
    - U? TMP100
      - Addr: `01` (PIN) --> `0x4A` (ADDR)

#### Understand how a record works

Take `record(ao, "$(P)$(R):HV")` as an example.

- Record definition
```
record(ao, "$(P)$(R):HV") {
  field(DESC, "HV")
  field(DTYP, "I2C D-A Converter")
  field(OUT,"#C0 S5 @12, 0") 
  field(LINR,"LINEAR")
  field(EGUF,"500.0")
  field(EGUL,"0.0")
  field(HOPR,"200.0")
  field(LOPR,"0.0")
  field(SCAN,"1 second")
  field(OROC,"2")
  field(PREC,"3")
  field(EGU,"Volts")
}
```

According to `DTYP` (`I2C D-A Converter`) -->

- `zDDMInc.dbd`

`device(ao,VME_IO,devAoI2C,"I2C D-A Converter")`

Look up `devAoI2C`

- `devI2C.c`

```
aoI2Cdset devAoI2C = {
    6,
    NULL,
    NULL,
    init_ao,
    NULL,
    write_ao,
    NULL};
```

where

```
typedef struct
{
  long number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
  DEVSUPFUN special_linconv;
} aoI2Cdset;
```

- `write_ao`

```c
static long write_ao(void *precord)
{
  aoRecord *pao = (aoRecord *)precord;
  long status;
  unsigned long rawVal = 0;
  int card, reg;
  I2CDpvt *pPvt = (I2CDpvt *)pao->dpvt;

  card = pao->out.value.vmeio.card;
  reg = pao->out.value.vmeio.signal;

  if (WriteDac(card, reg, pao->rval) == OK)
  {
    if (ReadDac(card, reg, &rawVal) == OK)
    {
      pao->rbv = rawVal;
      if (devI2CDebug >= 20)
        errlogPrintf("write_ao: card %i  reg %i rawVal %i\n\r", card, reg, rawVal);

      /* here is where we do the sign extensions for Bipolar.... */
      if (pPvt->type == 1)
      {
        if (pao->rbv & (2 << (pPvt->nbit - 2)))
          pao->rbv |= ((2 << 31) - (2 << (pPvt->nbit - 2))) * 2;
      }
      return (0);
    }
  }
  /* Set an alarm for the record */
  recGblSetSevr(pao, WRITE_ALARM, INVALID_ALARM);
  return (2);
}
```

It calls `WriteDac()` and `ReadDac`.

- `WriteDac()`

```c
int WriteDac(int card, int channel, int val)
{
  int dacaddr = cards[card].base; // The I2C address of the DAC
  char buf[3] = {0};
  char filename[20];
  int bytesWritten;
  int test;
  short int dacWord;

  dacWord = val;
  if (dacWord > 4095)
    dacWord = 4095;
  if (dacWord < 0)
    dacWord = 0;
  if (devI2CDebug >= 20)
  {
    errlogPrintf("Set DAC: %i\n", val);
    errlogPrintf("DAC Word: %d   (0x%x)\n", dacWord, dacWord);
  }
  sprintf(filename, "/dev/i2c-1");
  FASTLOCK(&I2C_lock_1);
  if ((I2Cdev = open(filename, O_RDWR)) < 0)
  {
    printf("Failed to open the bus.");
    FASTUNLOCK(&I2C_lock_1);
    exit(1);
  }

  if (ioctl(I2Cdev, I2C_SLAVE, dacaddr) < 0)
  {
    errlogPrintf("Failed to acquire bus access and/or talk to slave.\n");
    close(I2Cdev);
    FASTUNLOCK(&I2C_lock_1);
    return (-1);
  }
  buf[0] = 0x30 + channel; // Command Access Byte
  buf[1] = (char)((dacWord & 0x0FF0) >> 4);
  buf[2] = (char)((dacWord & 0x000F) << 4);
  //    printf("MSB: %x    LSB: %x\n",buf[1],buf[2]);

  bytesWritten = write(I2Cdev, buf, 3);
  if (devI2CDebug >= 20)
    errlogPrintf("Chip %i DAC %i Written...  Bytes Written : %d\n", card, channel, bytesWritten);
  close(I2Cdev);
  FASTUNLOCK(&I2C_lock_1);
  return (OK);
}
```

#### Summery

PV | TYPE | DTYP | INP | ADDR | CHAN | CHIP
:-:|:-:|:-:|:-:|:-:|:-:|:-:
HV | AO | `I2C D-A Converter` | `#C0 S5 @12, 0` |  |  | DAC7678
P1 | AO | `I2C D-A Converter` | `#C0 S6 @12, 0` |  |  | DAC7678
P2 | AO | `I2C D-A Converter` | `#C0 S2 @12, 0` |  |  | DAC7678
HV_RBV | AI | `I2C A-D Converter` | `#C1 S4 @12, 0` |  |  | LTC2309
HV_CUR | AI | `I2C A-D Converter` | `#C1 S5 @12, 0` |  |  | LTC2309
P1_CUR | AI | `I2C A-D Converter` | `#C1 S6 @12, 0` |  |  | LTC2309
P2_CUR | AI | `I2C A-D Converter` | `#C1 S7 @12, 0` |  |  | LTC2309
LK_MON | AI | `I2C A-D Converter` | `#C2 S0 @12, 0` |  |  | LTC2309
TEMP1 | AI | `Tmp100 A-D Converter` | `@72` |  |  | TMP100
TEMP2 | AI | `Tmp100 A-D Converter` | `@73` |  |  | TMP100
TEMP3 | AI | `Tmp100 A-D Converter` | `@74` |  |  | TMP100
ztemp | AI | `ZTMP A-D Converter` | `@0` | | | Zynq



## Appendix

### A1 UDP send and wait for response in IOC

```C++
#include <epicsThread.h>
#include <epicsEvent.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PORT 12345
#define TIMEOUT 5 // Timeout in seconds

// Global event
epicsEventId responseEvent;

// Function to handle UDP reception
void udpReceiver() {
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    char buffer[1024];
    socklen_t addr_len = sizeof(client_addr);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        return;
    }

    while (1) {
        int n = recvfrom(sockfd, (char *)buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len);
        if (n > 0) {
            buffer[n] = '\0';
            printf("Receved: %s\n", buffer);

            // Match the expected packet (e.g., check for a specific identifier)
            if (strstr(buffer, "Expected Response") != NULL) {
                // Signal that the expected response was received
                epicsEventSignal( responseEvent );
            }
        }
    }

    close(sockfd);
}

// Function to send a read request and wait for the response
void sendReadRequest() {
    int sockfd;
    struct sockaddr_in server_addr;
    char *readRequest = "Read Request";

    // Create event
    responseEvent = epicsEventCreate(epicsEventEmpty);

    // Create socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Address of the UDP receiver
    server_addr.sin_port = htons(PORT);

    // Send read request
    sendto( sockfd,
            readRequest,
            strlen(readRequest),
            0,
            (const struct sockaddr *)&server_addr,
            sizeof(server_addr)
          );

    // Wait for response (with timeout)
    int result = epicsEventWaitWithTimeout( responseEvent, TIMEOUT );
    if ( result == epicsEventWaitTimeout )
    {
        printf("Timeout waiting for response\n");
    }
    else
    {
        printf("Response received\n");
    }

    close(sockfd);
    epicsEventDestroy(responseEvent);
}

int main() {
    // Start the UDP receiver in a separate thread
    epicsThreadCreate( "udpReceiver",
                       epicsThreadPriorityMedium,
                       epicsThreadGetStackSize(epicsThreadStackSmall),
                       (EPICSTHREADFUNC)udpReceiver, NULL
                     );

    // Send read request and wait for the response
    sendReadRequest();

    return 0;
}
```




### A2  Records -> Driver parameters

#### A2.1 DAC

##### Enable internal reference

Write 0x80'00'10.

##### `:HV`

- Record

```
record(ao, "$(P)$(R):HV") {
  field(DESC, "HV")
  field(DTYP, "I2C D-A Converter")
  field(OUT,"#C0 S5 @12, 0") 
  field(LINR,"LINEAR")
  field(EGUF,"500.0")
  field(EGUL,"0.0")
  field(HOPR,"200.0")
  field(LOPR,"0.0")
#  field(SCAN,"1 second")
  field(OROC,"2")
  field(PREC,"3")
  field(EGU,"Volts")
}
```

- PV write:

```
caput det1:HV 1
```

- IOC output

`write_ao` -> `WriteDac` -> `ReadDac`

  - buf[0] = 0x30 + chan
  - buf[1] = (val & 0xFF0) >> 4
  - buf[2] = (val & 0x00F) << 4

PV Value | Card | Reg | DAC ADDR | WriteDac Val | Chan | ReadDac Val | buf[0] | buf[1] | buf[2]
:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:
1 | 0 | 5 | 74 | 8 | 5 | 8 | 0x35 | 0x0 | 0x80
2 | 0 | 5 | 74 | 16 | 5 | 16 | 0x35 | 0x1 | 0x0
5 | 0 | 5 | 74 | 33 | 5 | 33 | 0x35 | 0x2 | 0x10

##### `:P1`

- Record

```
record(ao, "$(P)$(R):P1") {
  field(DESC, "Peltier #1")
  field(DTYP, "I2C D-A Converter")
  field(OUT,"#C0 S6 @12, 0")
  field(LINR,"LINEAR")
  field(EGUF,"5.0")
  field(EGUL,"0.0")
  field(HOPR,"5.0")
  field(LOPR,"0.0")
  field(PREC,"3")
  field(EGU,"Volts")
}
```

- PV write:

```
caput det1:P1 1
```

- IOC output

`write_ao` -> `WriteDac` -> `ReadDac`

  - buf[0] = 0x30 + chan
  - buf[1] = (val & 0xFF0) >> 4
  - buf[2] = (val & 0x00F) << 4

PV Value | Card | Reg | DAC ADDR | WriteDac Val | Chan | ReadDac Val | buf[0] | buf[1] | buf[2]
:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:
1 | 0 | 6 | 74 | 819 | 6 | 819 | 0x36 | 0x33 | 0x30
2 | 0 | 6 | 74 | 1638 | 6 | 1638 | 0x36 | 0x66 | 0x60
5 | 0 | 6 | 74 | 33 | 5 | 33 | 0x32 | 0x2 | 0x10

##### `:P2`

- Record

```
record(ao, "$(P)$(R):P2") {
  field(DESC, "Peltier #2")
  field(DTYP, "I2C D-A Converter")
  field(OUT,"#C0 S2 @12, 0")
  field(LINR,"LINEAR")
  field(EGUF,"5.0")
  field(EGUL,"0.0")
  field(HOPR,"5.0")
  field(LOPR,"0.0")
  field(PREC,"3")
  field(EGU,"Volts")
}
```

- PV write:

```
caput det1:P2 1
```

- IOC output

`write_ao` -> `WriteDac` -> `ReadDac`

  - buf[0] = 0x30 + chan
  - buf[1] = (val & 0xFF0) >> 4
  - buf[2] = (val & 0x00F) << 4

PV Value | Card | Reg | DAC ADDR | WriteDac Val | Chan | ReadDac Val | buf[0] | buf[1] | buf[2]
:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:
1 | 0 | 2 | 74 | 819 | 2 | 819 | 0x32 | 0x33 | 0x30
2 | 0 | 2 | 74 | 1638 | 2 | 1638 | 0x32 | 0x66 | 0x60
5 | 0 | 2 | 74 | 33 | 2 | 4095 | 0x32 | 0xFF | 0xF0

PV | Card | Reg/Chan | DAC ADDR | cntrlword
:-:|:-:|:-:|:-:
`:HV` | 0 | 5 | 74
`:P1` | 0 | 6 | 74
`:HV` | 0 | 2 | 74


##### Driver implementation

```c++
class PSI2C
{
private:
    uint8_t  psi2c_index_;
    XIicPs   psi2c_instance_;
    uint32_t sclk_rate_;
    SemaphoreHandle_t pmutex_;


public:
    PSI2C( uint32_t sclk_rate );

    int write( uint16_t slave_addr, uint8_t *buffer, int length );

    int read(uint16_t slave_addr, uint8_t *buffer, int length);
};


//============================================
// PSI2C constructor.
//============================================
PSI2C( uint32_t sclk_rate ) : sclk_rate_(sclk_rate)
{
    // init
    XIicPs_Config *pconfig;
    int Status;

    pmutex_ = xSemaphoreCreateMutex();

    // Look up the configuration
    pconfig = XIicPs_LookupConfig(I2C_DEVICE_ID);
    if (pconfig == NULL) return XST_FAILURE;

    // Initialize the I2C driver instance
    Status = XIicPs_CfgInitialize( &psi2c_instance_, pconfig, Config->BaseAddress );
    if (Status != XST_SUCCESS) return XST_FAILURE;

    // Set the I2C clock speed
    XIicPs_SetSClk(&psi2c_instance_, sclk_rate_);
}
//============================================

//============================================
// PSI2C write.
//============================================
int PSI2C::write( uint16_t slave_addr, uint8_t *buffer, int length )
{
    int status;

    if (xSemaphoreTake( pmutex_, pdMS_TO_TICKS(100) ) == pdTRUE)
    {
        status = XIicPs_MasterSendPolled( &psi2c_instance_, buffer, length, slave_addr );
        if ( status != XST_SUCCESS )
            return XST_FAILURE;

        // Wait until the bus is idle
        while ( XIicPs_BusIsBusy( &psi2c_instance_ ) );
        xSemaphoreGive( pmutex_ );
    }

    return XST_SUCCESS;
}
//============================================

//============================================
// PSI2C read.
//============================================
int PSI2C::read(uint16_t slave_addr, uint8_t *buffer, int length)
{
    int status;

    if ( xSemaphoreTake( pmutex_, pdMS_TO_TICKS(100)) == pdTRUE )
    {
        status = XIicPs_MasterRecvPolled( &psi2c_instance_, buffer, length, slave_addr );
        if ( status != XST_SUCCESS ) return XST_FAILURE;

        // Wait until the bus is idle
        while ( XIicPs_BusIsBusy( &psi2c_instance_ ) );
        xSemaphoreGive( pmutex_ );
    }

    return XST_SUCCESS;
}
```

```c++
class DAC7678
{
private:
    uint8_t i2c_addr_;
    req_queue_;

    PSI2CReq psi2c_req_;
    std::map<int, int> chan_assign_;  // stores <variable:channel>
                                      // defined by detector and passed to the constructor

public:
    DAC7678( uint8_t i2c_addr, psi2c_req_queue, std::map<int, char> chan_assign);

    ~DAC7678() = default;

    void write( uint8_t variable, uint16_t data );

    void read( uint8_t chan, uint16_t data );
};


//============================================
// DAC7678 constructor.
//============================================
DAC7678::DAC7678( uint8_t i2c_addr, psi2c_req_queue, std::map<int, char> chan_assign)
                : i2c_addr_(i2c_addr)
                , req_queue_(req_queue)
                , chan_assign_(chan_assign)
{
    // enable internal reference
    psi2c_req_.addr = i2c_addr_;
    psi2c_req_.data[0] = 0x80;
    psi2c_req_.data[1] = 0x00;
    psi2c_req_.data[2] = 0x10;

    // send req to queue
}

//============================================
// DAC7678 write.
// Requires the ID of the physical variable.
//============================================
DAC7678::write( uint8_t variable, uint16_t data )
{
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        psi2c_req_.data[0] = 0x30 + chan_assign[var];
        psi2c_req_.data[1] = static_cast<uint8_t>(data >> 4);
        psi2c_req_.data[2] = static_cast<uint8_t>((data && 0x0F) << 4);
        psi2c_req_.length = 3;
        psi2c_req_.read = 0;
                
        // send req to queue
    }
    else
    {
        // error
    }
}
//============================================

//============================================
// DAC7678 read.
// Requires the ID of the physical variable.
//============================================
DAC7678::read( uint8_t chan )
{
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        psi2c_req_.data[0] = 0x10 + chan_assign[var];
        psi2c_req_.read = 1;
        
        // send req to queue
    }
}
//============================================
```

#### A2.2 ADC

##### - `:HV_RBV`

- Record

```
record(ai, "$(P)$(R):HV_RBV") {
  field(DESC, "HV readback")
  field(DTYP, "I2C A-D Converter")
  field(INP,"#C1 S4 @12, 0") 
  field(LINR,"LINEAR")
#  field(SCAN,"10 second")
  field(EGUF,"500.0")
  field(EGUL,"0.0")
  field(HOPR,"200.0")
  field(LOPR,"0.0")
  field(PREC,"1")
  field(EGU,"Volts")
}
```

- IOC output

`read_ai` -> `ReadAdc`

  - val = buf[0] << 8 | buf[1] >> 4

Card | Reg | DAC ADDR | cntrlword | read val | buf[0] | buf[1]
:-:|:-:|:-:|:-:|:-:|:-:|:-:
1 | 4 | 8 | 168 | 34 | 0x2 | 0x20

##### - `:HV_CUR`

Card: 1
reg: 5
Chan: 5
dacaddr: 8
cntrlword: 232

##### - `:P1_CUR`

Card: 1
reg: 6
chan 6
dacaddr: 8
cntrlword: 184

##### - `:P2_CUR`

Card: 1
reg: 7
chan 7
dacaddr: 8
cntrlword: 248

i2c address: 0x8
chan: 4/5/6/7
cntrlwords are fixed for channs

PV | Card | Reg/Chan | DAC ADDR | cntrlword
:-:|:-:|:-:|:-:|:-:
`:HV_RBV` | 1 | 4 | 8 | 0xA8
`:HV_CUR` | 1 | 5 | 8 | 0xE8
`:P1_CUR` | 1 | 6 | 8 | 0xB8
`:P2_CUR` | 1 | 7 | 8 | 0xF8


##### - Driver

```c++
class LTC2309
{
private:
    uint8_t i2c_addr_;
    uint8_t single;
    req_queue_;

    PSI2CReq psi2c_req_;
    std::map<int, int> chan_assign_;  // stores <variable:channel>
                                      // defined by detector and passed to the constructor

public:
    LTC2309( uint8_t i2c_addr
           , bool is_single_ended
           , psi2c_req_queue
           , std::map<int, char> chan_assign)
        : i2c_addr_(i2c_addr)
        , signle( is_single_ended ? 0x8 : 0 )
        , req_queue_(req_queue)
        , chan_assign_(chan_assign)
    {}

    ~LTC2309() = default;

    void read( uint8_t chan );
};

//============================================
// LTC2309 read.
// Requires the ID of the physical variable.
//============================================
void LTC2309::read( uint8_t chan )
{
    char buff[2];
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        char chan = chan_assign[var];
        req.data[0] = single | ((chan & 0x1)<<2) | ((chan & 0x6)>>1);
        req.length = 1;
        req.read = 0;
        
        // send req to queue

        req.data[0] = 0;
        req.data[1] = 0;

        req.length = 2;
        req.read = 1;

        // send req to queue

    }
}
//============================================
```

#### A2.3 Tmp100

- PVs: `:Temp1`, `:Temp2`, `:Temp3`
  - `EGUF` - Engineer Units Full: 4.096
  - `EGUL` - Engineer Units Low: 0
  - `HOPR` - High Operating Range: 4.096
  - `ESLO` - Raw to EGU Slope: 0.0625
  - `EOFF` - Raw to EGU Offset: 0

##### Driver

```c++
class Tmp100
{
private:
    uint8_t i2c_addr_;
    req_queue_;

    PSI2CReq psi2c_req_;
    std::map<std::string, char> addr_assign_;  // stores <variable:address>
                                      // defined by detector and passed to the constructor

public:
    Tmp100( uint8_t i2c_addr
           , psi2c_req_queue )
        : i2c_addr_(i2c_addr)
        , req_queue_(req_queue)
        , std::map<std::string, char> addr_assign
    {}

    ~Tmp100() = default;

    void read( uint16_t* data );
};

//============================================
// LTC2309 read.
// Requires the ID of the physical variable.
//============================================
void Tmp100::read( std::string dev )
{
    char buff[2];
    if( chan_assign_.find(dev) != chan_assign_.end() )
    {
        char chan = chan_assign[var];
        req.data[0] = single | ((chan & 0x1)<<2) | ((chan & 0x6)>>1);
        req.length = 1;
        req.read = 0;
        
        // send req to queue

        req.data[0] = 0;
        req.data[1] = 0;

        req.length = 2;
        req.read = 1;

        // send req to queue

    }
}
//============================================
```

### A3 Data Types and Function calls

#### A3.1 Data Types

##### A3.1.1 Requests

- `RegisterMultiAccessRequestData` / `RegisterMultiAccessRequestDataStruct`

```c++
union RegisterMultiAccessRequestDataStruct
{   
    ZddmArm    zddm_arm_data;
    Ad9252Cfg  ad9252_cfg_data;
};  
using RegisterMultiAccessRequestData = RegisterMultiAccessRequestDataStruct;
```

- `RegisterMultiAccessRequest` / `RegisterMultiAccessRequestStruct`

```c++
struct RegisterMultiAccessRequestStruct
{   
    uint16_t  op; 
    char      data[sizeof(RegisterMultiAccessRequestData)];
};  
using RegisterMultiAccessRequest = RegisterMultiAccessRequestStruct;
```

##### A3.1.2 Responses

##### A3.1.3 Queues

```c++
QueueHandle_t
QueueSetMemberHandle_t
QueueSetHandle_t

```

##### A3.1.4 Tasks

```c++
TaskHandle_t

```

#### A3.1.3 Messages

- `Ad9252Cfg` / `Ad9252CfgStruct`
```c++
struct Ad9252CfgStruct
{
    uint16_t  chip_num;
    uint16_t  addr;
    uint32_t  data;
};
using Ad9252Cfg = Ad9252CfgStruct;
```

- `ZddmArm` / `ZddmArmStruct`

```c++
struct ZddmArmStruct
{
    uint16_t  mode;
    uint16_t  val;
};
using ZddmArm = ZddmArmStruct;
```

- `UdpRxMsgPayloadStruct`/`UdpRxMsgPayload`

```c++
union UdpRxMsgPayloadStruct
{   
    uint32_t   reg_single_acc_req_data;
    uint32_t   loads[12][14];
    ZddmArm    zddm_arm_data;
    Ad9252Cfg  ad9252_cfg_data;
    uint32_t   i2c_acc_req_data;
    uint32_t   xadc_acc_req_data;
};  
using UdpRxMsgPayload = UdpRxMsgPayloadStruct;
```

- `UdpRxMsgStruct`/`UdpRxMsg`

```c++
struct UdpRxMsgStruct
{   
    uint16_t  id; 
    uint16_t  op; 
    char      payload[sizeof(DerivedNetwork::UdpRxMsgPayload)];
};  
using UdpRxMsg = UdpRxMsgStruct;

```


- `UdpTxMsgStruct`/`UdpTxMsg`

```c++
struct UdpTxMsgStruct
{
    uint16_t  id;
    uint16_t  op;
    char      payload[sizeof(DerivedNetwork::UdpTxMsgPayload)];
};
using UdpTxMsg = UdpTxMsgStruct;
```

- `UdpTxMsgPayloadStruct`/`UdpTxMsgPayload`

```c++
union UdpTxMsgPayloadStruct
{
    uint32_t  register_single_access_response_data;
    uint32_t  psi2c_access_response_data;
    uint32_t  psxadc_access_response_data;
};
using UdpTxMsgPayload = UdpTxMsgPayloadStruct;
```


#### A3.2 Function Calls

- ##### Queue
```c++
xQueueReceive( static_cast<QueueHandle_t*>(pvParameters),
						   &req,
						   portMAX_DELAY );

xQueueSend( single_register_access_request_queue,
           	req,
            0UL );
```

- ##### Task

```c++
// FreeRTOS function call
xTaskCreate( udp_rx_task,
             ( const char * ) "UDP_RX",
    				 configMINIMAL_STACK_SIZE,
		    		 NULL,
				     tskIDLE_PRIORITY,
				     &udp_rx_task_handle_ );
```

```c++
// A free function task_wrapper
void task_wrapper(void* pvParameters)
{
    auto* task_func = static_cast<std::function<void()>*>( pvParameters );
    if (task_func)
    {
        (*task_func)();  // Call the actual function
    }
    vTaskDelete(nullptr);
}

// Use case
class MyClass
{
public:
    MyClass()
    {
        task_func = [this]() { task_function(); };
        xTaskCreate( task_wrapper, "MyTask", 1000, &task_func, 1, &taskHandle );
    }

private:
    TaskHandle_t taskHandle;
    std::function<void()> task_func;

    friend void task_wrap( void* pvParameters );
};
```


