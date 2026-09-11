/** @file cpu1_board.c Shared-peripheral routing and multicore boot. */

#include "device.h"
#include "driverlib.h"

static void cpu1_setup_ethernet_gpio(void)
{
    SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);
    GPIO_setPinConfig(GPIO_105_ENET_MDIO_CLK);
    GPIO_setPinConfig(GPIO_106_ENET_MDIO_DATA);
    GPIO_setPinConfig(GPIO_109_ENET_MII_CRS);
    GPIO_setPinConfig(GPIO_110_ENET_MII_COL);
    GPIO_setPinConfig(GPIO_75_ENET_MII_TX_DATA0);
    GPIO_setPinConfig(GPIO_122_ENET_MII_TX_DATA1);
    GPIO_setPinConfig(GPIO_123_ENET_MII_TX_DATA2);
    GPIO_setPinConfig(GPIO_124_ENET_MII_TX_DATA3);
    GPIO_setPinConfig(GPIO_118_ENET_MII_TX_EN);
    GPIO_setPinConfig(GPIO_114_ENET_MII_RX_DATA0);
    GPIO_setPinConfig(GPIO_115_ENET_MII_RX_DATA1);
    GPIO_setPinConfig(GPIO_116_ENET_MII_RX_DATA2);
    GPIO_setPinConfig(GPIO_117_ENET_MII_RX_DATA3);
    GPIO_setPinConfig(GPIO_113_ENET_MII_RX_ERR);
    GPIO_setPinConfig(GPIO_112_ENET_MII_RX_DV);
    GPIO_setPinConfig(GPIO_44_ENET_MII_TX_CLK);
    GPIO_setPinConfig(GPIO_111_ENET_MII_RX_CLK);
    GPIO_setDirectionMode(108U, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(108U, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(108U, 1U);
    GPIO_setDirectionMode(119U, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(119U, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(119U, 1U);
}

static void cpu1_setup_ethercat_gpio(void)
{
    GPIO_setPinConfig(GPIO_154_ESC_PHY_CLK);
    GPIO_setPinConfig(GPIO_155_ESC_PHY_RESETN);
    GPIO_setPinConfig(GPIO_150_ESC_I2C_SDA);
    GPIO_setPadConfig(150U, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_151_ESC_I2C_SCL);
    GPIO_setPadConfig(151U, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_158_ESC_TX0_DATA0);
    GPIO_setPinConfig(GPIO_159_ESC_TX0_DATA1);
    GPIO_setPinConfig(GPIO_160_ESC_TX0_DATA2);
    GPIO_setPinConfig(GPIO_161_ESC_TX0_DATA3);
    GPIO_setPinConfig(GPIO_165_ESC_RX0_DATA0);
    GPIO_setPinConfig(GPIO_166_ESC_RX0_DATA1);
    GPIO_setPinConfig(GPIO_167_ESC_RX0_DATA2);
    GPIO_setPinConfig(GPIO_168_ESC_RX0_DATA3);
    GPIO_setPinConfig(GPIO_156_ESC_TX0_ENA);
    GPIO_setPinConfig(GPIO_162_ESC_RX0_DV);
    GPIO_setPinConfig(GPIO_164_ESC_RX0_ERR);
    GPIO_setPinConfig(GPIO_157_ESC_TX0_CLK);
    GPIO_setPinConfig(GPIO_163_ESC_RX0_CLK);
    GPIO_setPinConfig(GPIO_148_ESC_PHY0_LINKSTATUS);
    GPIO_setPinConfig(GPIO_143_ESC_LED_LINK0_ACTIVE);
    GPIO_setPinConfig(GPIO_152_ESC_MDIO_CLK);
    GPIO_setPinConfig(GPIO_153_ESC_MDIO_DATA);
    GPIO_setPinConfig(GPIO_131_ESC_TX1_DATA0);
    GPIO_setPinConfig(GPIO_132_ESC_TX1_DATA1);
    GPIO_setPinConfig(GPIO_134_ESC_TX1_DATA2);
    GPIO_setPinConfig(GPIO_135_ESC_TX1_DATA3);
    GPIO_setPinConfig(GPIO_139_ESC_RX1_DATA0);
    GPIO_setPinConfig(GPIO_140_ESC_RX1_DATA1);
    GPIO_setPinConfig(GPIO_141_ESC_RX1_DATA2);
    GPIO_setPinConfig(GPIO_142_ESC_RX1_DATA3);
    GPIO_setPinConfig(GPIO_129_ESC_TX1_ENA);
    GPIO_setPinConfig(GPIO_136_ESC_RX1_DV);
    GPIO_setPinConfig(GPIO_138_ESC_RX1_ERR);
    GPIO_setPinConfig(GPIO_130_ESC_TX1_CLK);
    GPIO_setPinConfig(GPIO_137_ESC_RX1_CLK);
    GPIO_setPinConfig(GPIO_149_ESC_PHY1_LINKSTATUS);
    GPIO_setPinConfig(GPIO_144_ESC_LED_LINK1_ACTIVE);
    GPIO_setPinConfig(GPIO_125_ESC_LATCH0);
    GPIO_setPinConfig(GPIO_126_ESC_LATCH1);
    GPIO_setPinConfig(GPIO_127_ESC_SYNC0);
    GPIO_setPinConfig(GPIO_128_ESC_SYNC1);
}

void cpu1_board_start_multicore(void)
{
    /* CPU1 owns the boot-time allocation registers.  Communication runtime
     * (EtherCAT, USB and Ethernet) belongs to CM after this handoff. */
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAT);
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_ETHERCAT, 1U);
    SysCtl_setECatClk(SYSCTL_ECATCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL, 1U);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAT);
    cpu1_setup_ethercat_gpio();

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_USBA);
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1U);
    cpu1_setup_ethernet_gpio();

#ifdef _FLASH
    Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCPU2(BOOTMODE_BOOT_TO_M0RAM);
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
}
