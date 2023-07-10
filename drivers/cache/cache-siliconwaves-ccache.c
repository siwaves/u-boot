// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 siliconwaves
 */

#include <common.h>
#include <cache.h>
#include <dm.h>
#include <asm/io.h>
#include <dm/device.h>
#include <linux/bitfield.h>

#define SILICONWAVES_CCACHE_CONFIG		0x000
#define SILICONWAVES_CCACHE_CONFIG_WAYS	GENMASK(15, 8)

#define SILICONWAVES_CCACHE_WAY_ENABLE	0x008

struct siliconwaves_ccache {
	void __iomem *base;
};

static int siliconwaves_ccache_enable(struct udevice *dev)
{
	struct siliconwaves_ccache *priv = dev_get_priv(dev);
	u32 config;
	u32 ways;

	/* Enable all ways of composable cache */
	config = readl(priv->base + SILICONWAVES_CCACHE_CONFIG);
	ways = FIELD_GET(SILICONWAVES_CCACHE_CONFIG_WAYS, config);
	printf("config = 0x%x, ways 0x%x\n", config, ways);
	writel(ways - 1, priv->base + SILICONWAVES_CCACHE_WAY_ENABLE);

	return 0; 
}

static int siliconwaves_ccache_get_info(struct udevice *dev, struct cache_info *info)
{
	struct siliconwaves_ccache *priv = dev_get_priv(dev);

	info->base = (uintptr_t)priv->base;

	return 0;
}

static const struct cache_ops siliconwaves_ccache_ops = {
	.enable = siliconwaves_ccache_enable,
	.get_info = siliconwaves_ccache_get_info,
};

static int siliconwaves_ccache_probe(struct udevice *dev)
{
	struct siliconwaves_ccache *priv = dev_get_priv(dev);

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	return 0;
}

static const struct udevice_id siliconwaves_ccache_ids[] = {
	{ .compatible = "siliconwaves,w3k-ccache" },
	{}
};

U_BOOT_DRIVER(siliconwaves_ccache) = {
	.name = "siliconwaves_ccache",
	.id = UCLASS_CACHE,
	.of_match = siliconwaves_ccache_ids,
	.probe = siliconwaves_ccache_probe,
	.priv_auto = sizeof(struct siliconwaves_ccache),
	.ops = &siliconwaves_ccache_ops,
};
