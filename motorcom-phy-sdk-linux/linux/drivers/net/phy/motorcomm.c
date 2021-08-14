/*
 * drivers/net/phy/motorcomm.c
 *
 * Driver for Motorcomm PHYs
 *
 * Author: Leilei Zhao <leilei.zhao@motorcomm.com>
 *
 * Copyright (c) 2019 Motorcomm, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support : Motorcomm Phys:
 *		Giga phys: yt8511, yt8521
 *		100/10 Phys : yt8512, yt8512b, yt8510
 *		Automotive 100Mb Phys : yt8010
 *		Automotive 100/10 hyper range Phys: yt8510
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/motorcomm_phy.h>
#include <linux/of.h>
#include <linux/clk.h>
#define GMAC_CLOCK_INPUT_NEEDED 1	//some GMAC need clock input from PHY, for eg., 125M

static int ytphy_read_ext(struct phy_device *phydev, u32 regnum)
{
	int ret;
	int val;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_DEBUG_DATA);

	return val;
}

static int ytphy_write_ext(struct phy_device *phydev, u32 regnum, u16 val)
{
	int ret;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, REG_DEBUG_DATA, val);

	return ret;
}

static int yt8010_config_aneg(struct phy_device *phydev)
{
	phydev->speed = SPEED_100;
	return 0;
}

static int yt8512_clk_init(struct phy_device *phydev)
{
	int ret;
	int val;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_AFE_PLL);
	if (val < 0)
		return val;

	val |= YT8512_CONFIG_PLL_REFCLK_SEL_EN;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_AFE_PLL, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_EXTEND_COMBO);
	if (val < 0)
		return val;

	val |= YT8512_CONTROL1_RMII_EN;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_EXTEND_COMBO, val);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, MII_BMCR);
	if (val < 0)
		return val;

	val |= YT_SOFTWARE_RESET;
	ret = phy_write(phydev, MII_BMCR, val);

	return ret;
}

static int yt8512_led_init(struct phy_device *phydev)
{
	int ret;
	int val;
	int mask;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED0);
	if (val < 0)
		return val;

	val |= YT8512_LED0_ACT_BLK_IND;

	mask = YT8512_LED0_DIS_LED_AN_TRY | YT8512_LED0_BT_BLK_EN |
		YT8512_LED0_HT_BLK_EN | YT8512_LED0_COL_BLK_EN |
		YT8512_LED0_BT_ON_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_LED0, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED1);
	if (val < 0)
		return val;

	val |= YT8512_LED1_BT_ON_EN;

	mask = YT8512_LED1_TXACT_BLK_EN | YT8512_LED1_RXACT_BLK_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_LED1_BT_ON_EN, val);

	return ret;
}

static int yt8512_config_init(struct phy_device *phydev)
{
	int ret;
	int val;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	ret = yt8512_clk_init(phydev);
	if (ret < 0)
		return ret;

	ret = yt8512_led_init(phydev);

	/* disable auto sleep */
	val = ytphy_read_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(YT8512_EN_SLEEP_SW_BIT));

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;

	return ret;
}

static int yt8512_read_status(struct phy_device *phydev)
{
	int ret;
	int val;
	int speed, speed_mode, duplex;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	duplex = (val & YT8512_DUPLEX) >> YT8512_DUPLEX_BIT;
	speed_mode = (val & YT8512_SPEED_MODE) >> YT8512_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		speed = SPEED_10;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 2:
	case 3:
	default:
		speed = SPEED_UNKNOWN;
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;

	return 0;
}

int yt8521_soft_reset(struct phy_device *phydev)
{
	int ret;

	ytphy_write_ext(phydev, 0xa000, 0);
	ret = genphy_soft_reset(phydev);
	if (ret < 0)
		return ret;

	ytphy_write_ext(phydev, 0xa000, 2);
	ret = genphy_soft_reset(phydev);
	if (ret < 0) {
		ytphy_write_ext(phydev, 0xa000, 0);
		return ret;
	}

	return 0;
}
#if GMAC_CLOCK_INPUT_NEEDED
static int ytphy_mii_rd_ext(struct mii_bus *bus, int phy_id, u32 regnum)
{
	int ret;
	int val;

	ret = bus->write(bus, phy_id, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = bus->read(bus, phy_id, REG_DEBUG_DATA);

	return val;
}

static int ytphy_mii_wr_ext(struct mii_bus *bus, int phy_id, u32 regnum, u16 val)
{
	int ret;

	ret = bus->write(bus, phy_id, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = bus->write(bus, phy_id, REG_DEBUG_DATA, val);

	return ret;
}

int yt8511_config_dis_txdelay(struct mii_bus *bus, int phy_id)
{
        int ret;
        int val;

        /* disable auto sleep */
        val = ytphy_mii_rd_ext(bus, phy_id, 0x27);
        if (val < 0)
                return val;

        val &= (~BIT(15));

        ret = ytphy_mii_wr_ext(bus, phy_id, 0x27, val);
        if (ret < 0)
                return ret;

        /* enable RXC clock when no wire plug */
        val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
        if (val < 0)
                return val;

        /* ext reg 0xc b[7:4]
	Tx Delay time = 150ps * N – 250ps
        */
        val &= ~(0xf << 4);
        ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);
        printk("yzhang..phy txdelay, val=%#08x\n",val);

        return ret;
}


int yt8511_config_out_125m(struct mii_bus *bus, int phy_id)
{
        int ret;
        int val;

        /* disable auto sleep */
        val = ytphy_mii_rd_ext(bus, phy_id, 0x27);
        if (val < 0)
                return val;

        val &= (~BIT(15));

        ret = ytphy_mii_wr_ext(bus, phy_id, 0x27, val);
        if (ret < 0)
                return ret;

        /* enable RXC clock when no wire plug */
        val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
        if (val < 0)
                return val;

        /* ext reg 0xc.b[2:1]
        00-----25M from pll;
        01---- 25M from xtl;(default)
        10-----62.5M from pll;
        11----125M from pll(here set to this value)
        */
        val |= (3 << 1);
        ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);
	printk("yzhang..phy clk out, val=%#08x\n",val);

#if 0	
	//configure to master	
	val = bus->read(bus, phy_id, 0x9/*master/slave config reg*/);
	val |= (0x3<<11); //to be manual config and force to be master
	ret = bus->write(bus, phy_id, 0x9, val); //take effect until phy soft reset
	if (ret < 0)
		return ret;

	printk("yzhang..phy to be master, val=%#08x\n",val);
#endif

        return ret;
}

static int yt8511_config_init(struct phy_device *phydev)
{
	int ret;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	//ret = yt8511_config_out_125m(phydev->bus, phydev->addr);

	//printk("yzhang..phy config init done, phy_id=%#08x,phyaddr=%d\n",phydev->phy_id, phydev->addr);
	return ret;
}

#endif /*GMAC_CLOCK_INPUT_NEEDED*/

#define YT8521_NUM_OF_FIBER_PHY	3
static unsigned long autoneg_phydev[YT8521_NUM_OF_FIBER_PHY];
static int autoneg_pre_val[YT8521_NUM_OF_FIBER_PHY];
static int autoneg_changed[YT8521_NUM_OF_FIBER_PHY];
static int autoneg_actual_num_phy = 0;

static int yt8521_config_init(struct phy_device *phydev)
{
	int ret;
	int val;

	/* start workaround for 8521 100m fiber init*/
	int i;
	
	for(i = 0; i < YT8521_NUM_OF_FIBER_PHY; i++)
	{
		if((autoneg_phydev[i]) && (autoneg_phydev[i] == (unsigned long)phydev->attached_dev))
		{
			autoneg_changed[i] = 0;
			break;
		}else if(0 == autoneg_phydev[i])
		{
			autoneg_phydev[i] = (unsigned long)phydev->attached_dev;
			autoneg_changed[i] = 0;
			autoneg_actual_num_phy++;
			printk("yzhnag..8521 fiber workaround init for eth%d,macbase=%#lx\n", i,autoneg_phydev[i]);
			break;
		}
	}
	/*end of workaround for 8521 */

	phydev->irq = PHY_POLL;

	ytphy_write_ext(phydev, 0xa000, 0);
	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	/* disable auto sleep */
	val = ytphy_read_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(YT8521_EN_SLEEP_SW_BIT));

	ret = ytphy_write_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;

	/* enable RXC clock when no wire plug */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, 0xc);
	if (val < 0)
		return val;
	val &= ~(1 << 12);
	ret = ytphy_write_ext(phydev, 0xc, val);
	if (ret < 0)
		return ret;
	printk (KERN_INFO "yzhang..8521 init call out...\n");
	return ret;
}

static int yt8521_adjust_status(struct phy_device *phydev, int val, int is_utp)
{
	int speed_mode, duplex;
	int speed = SPEED_UNKNOWN;

	//printk (KERN_INFO "yzhang..8521 status adjust call in...\n");
	duplex = (val & YT8512_DUPLEX) >> YT8521_DUPLEX_BIT;
	speed_mode = (val & YT8521_SPEED_MODE) >> YT8521_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		if (is_utp)
			speed = SPEED_10;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 2:
		speed = SPEED_1000;
		break;
	case 3:
		break;
	default:
		speed = SPEED_UNKNOWN;
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;
	//printk (KERN_INFO "yzhang..8521 status adjust call out,regval=0x%04x,mode=%s,speed=%dm...\n", val,is_utp?"utp":"fiber", phydev->speed);

	return 0;
}

#define YT8521_PHY_MODE_FIBER	1 //fiber mode only
#define YT8521_PHY_MODE_UTP		2 //utp mode only
#define YT8521_PHY_MODE_POLL	3 //fiber and utp, poll mode
#define YT8521_PHY_MODE_CURR	3

static int link_mode_8521 = 0; //0: no link; 1: utp; 32: fiber. traced that 1000m fiber uses 32.

int yt8521_aneg_done (struct phy_device *phydev)
{

	//printk("yzhang-phy..YT8521 AN_done callin,speed=%dm,linkmoded=%d\n", phydev->speed,link_mode_8521);

	if((32 == link_mode_8521) && (SPEED_100 == phydev->speed))
	{
		return 1/*link_mode_8521*/;
	}
#if 0
	else
	{
		val = ytphy_read_ext(phydev, 0xa000);
		if((2 == val/*fiber mode*/) && (SPEED_100 == phydev->speed))
		{
			val = phy_read(phydev, 1);//read status register 1,latched
			val = phy_read(phydev, 0x11);//read status register 0x11
			val = phy_read(phydev, 1);//read status register 1
			if((0 < val) && (0x4 & val)) return link_mode_8521; 
		}
	}
#endif
	return genphy_aneg_done(phydev);
}

static int yt8521_read_status(struct phy_device *phydev)
{
        int i;
	int ret;
	volatile int val, yt8521_fiber_latch_val, yt8521_fiber_curr_val;
	volatile int link;
	int link_utp = 0, link_fiber = 0;
#if 0	
	static int cnt_read_status = 0;
	struct net_device * p_tmp_netdev = phydev->attached_dev;

	if(phydev->attached_dev) {
	 	//printk("yzhang..phy attached to a macdev\n");
	}

	if(cnt_read_status)
	{
		cnt_read_status--;
		//printk (KERN_INFO "yzhang..8521 read status cnt=%d\n",cnt_read_status);
		if(0<cnt_read_status)
		{
			return 0;
		}

	}
	cnt_read_status = 10;
#endif
	//printk (KERN_INFO "yzhang..8521 read status call in\n");

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)
	/* reading UTP */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if (link) {
		link_utp = 1;
		link_mode_8521 = 1;
		yt8521_adjust_status(phydev, val, 1);
	} else {
		link_utp = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)
	/* reading Fiber */
	ret = ytphy_write_ext(phydev, 0xa000, 2);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;
	
	//printk (KERN_INFO "yzhang..8521 read fiber status=%04x,macbase=0x%08lx\n", val,(unsigned long)phydev->attached_dev);

	/* for fiber, from 1000m to 100m, there is not link down from 0x11, and check reg 1 to identify such case */	
	yt8521_fiber_latch_val = phy_read(phydev, MII_BMSR);
	yt8521_fiber_curr_val = phy_read(phydev, MII_BMSR);
	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if((link) && (yt8521_fiber_latch_val != yt8521_fiber_curr_val))
	{
		link = 0;
		printk (KERN_INFO "yzhang..8521 fiber link down detect,latch=%04x,curr=%04x\n", yt8521_fiber_latch_val,yt8521_fiber_curr_val);
	}
	
	if (link) {
		link_fiber = 1;
		yt8521_adjust_status(phydev, val, 0);
		link_mode_8521 = 32; //fiber mode

#if 0
        	/* start workaround for 8521 100m fiber autoneg */
        	for(i = 0; i < YT8521_NUM_OF_FIBER_PHY; i++)
        	{
			if(autoneg_phydev[i] == (unsigned long)phydev->attached_dev){
				break;
                	}
        	}
		//printk("yzhnag..8521 fiber workaround now is for eth%d,macbase=%#lx\n", i,autoneg_phydev[i]);

		if((0 == autoneg_changed[i]) && (SPEED_100 == phydev->speed))
		{
			autoneg_pre_val[i] = phydev->autoneg;
			autoneg_changed[i] = 1;
			phydev->autoneg = AUTONEG_DISABLE;
			printk("yzhang-phy workaround eth%d, fiber 100m,autoneg[pre=%d,now=%d],changed=%d\n",i,autoneg_pre_val[i],phydev->autoneg,autoneg_changed[i]);
		}else if((autoneg_changed[i]) && (SPEED_100 != phydev->speed))
		{
			phydev->autoneg = autoneg_pre_val[i];
			autoneg_changed[i] = 0;
			printk("yzhang-phy workaround eth%d, fiber 1000m,restore autoneg,%d,changed=%d\n",i,phydev->autoneg,autoneg_changed[i]);
		}
        	/*end of workaround for 8521 */
#endif

	} else {
		link_fiber = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)

	if (link_utp || link_fiber) {
		phydev->link = 1;
	} else {
		phydev->link = 0;
		link_mode_8521 = 0;
	}

	if (link_utp) {
		ytphy_write_ext(phydev, 0xa000, 0);
	}
	//printk (KERN_INFO "yzhang..8521 read status call out,link=%d,linkmode=%d\n", phydev->link, link_mode_8521 );

	return 0;
}

int yt8521_suspend(struct phy_device *phydev)
{
	int value;

	/* no need lock in 4.19 */
	/* mutex_lock(&phydev->lock); */

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 0);

	/* mutex_unlock(&phydev->lock); */

	return 0;
}

int yt8521_resume(struct phy_device *phydev)
{
	int value;

	/* no need lock in 4.19 */
	/* mutex_lock(&phydev->lock); */

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 0);

	/* mutex_unlock(&phydev->lock); */

	return 0;
}

static struct phy_driver ytphy_drvs[] = {
	{
		.phy_id         = PHY_ID_YT8010,
		.name           = "YT8010 Automotive Ethernet",
		.phy_id_mask    = MOTORCOMM_PHY_ID_MASK,
		.features       = PHY_BASIC_FEATURES,
		.flags          = PHY_HAS_INTERRUPT,
		.config_aneg    = yt8010_config_aneg,
		.config_init	= genphy_config_init,
		.read_status    = genphy_read_status,
	}, {
		.phy_id		= PHY_ID_YT8510,
		.name		= "YT8510 100/10Mb Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= genphy_config_init,
		.read_status	= genphy_read_status,
	}, {
		.phy_id		= PHY_ID_YT8511,
		.name		= "YT8511 Gigabit Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
#if GMAC_CLOCK_INPUT_NEEDED
		.config_init	= yt8511_config_init,
#else
		.config_init	= genphy_config_init,
#endif
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
		.phy_id		= PHY_ID_YT8512,
		.name		= "YT8512 Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8512_config_init,
		.read_status	= yt8512_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
		.phy_id		= PHY_ID_YT8512B,
		.name		= "YT8512B Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8512_config_init,
		.read_status	= yt8512_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
                .phy_id         = PHY_ID_YT8521,
                .name           = "YT8521 Ethernet",
                .phy_id_mask    = MOTORCOMM_PHY_ID_MASK,
                .features       = PHY_BASIC_FEATURES | PHY_GBIT_FEATURES,
                .flags          = PHY_POLL,
		.soft_reset	= yt8521_soft_reset,
                .config_aneg    = genphy_config_aneg,
                .aneg_done	= yt8521_aneg_done,
                .config_init    = yt8521_config_init,
                .read_status    = yt8521_read_status,
                .suspend        = yt8521_suspend,
                .resume         = yt8521_resume,
        },
};

module_phy_driver(ytphy_drvs);

MODULE_DESCRIPTION("Motorcomm PHY driver");
MODULE_AUTHOR("Leilei Zhao");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused motorcomm_tbl[] = {
	{ PHY_ID_YT8010, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8510, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8511, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512B, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8521, MOTORCOMM_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, motorcomm_tbl);

