/*---------------------------------------------------------------------------
 * 
 * compat.h 
 *     Copyright (c) 2012 Guenter Roeck <linux@roeck-us.net>
 *
 *---------------------------------------------------------------------------
 */

#ifndef COMPAT_H
#define COMPAT_H

#include <linux/version.h>

#ifndef clamp_val
#define clamp_val SENSORS_LIMIT
#endif

#ifndef request_muxed_region
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
#define request_muxed_region(start,n,name)	__request_region(&ioport_resource, (start), (n), (name))
#else
#define IORESOURCE_MUXED	0x00400000
#define request_muxed_region(start,n,name)	__request_region(&ioport_resource, (start), (n), (name), IORESOURCE_MUXED)
#endif
#endif

/* Red Hat EL5 includes backports of these functions, so we can't redefine
 * our own. */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 24)
#if !(defined RHEL_MAJOR && RHEL_MAJOR == 5 && RHEL_MINOR >= 5)
static inline int strict_strtoul(const char *cp, unsigned int base,
                                 unsigned long *res)
{
	*res = simple_strtoul(cp, NULL, base);
	return 0;
}

static inline int strict_strtol(const char *cp, unsigned int base, long *res)
{
	*res = simple_strtol(cp, NULL, base);
	return 0;
}
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
#define kstrtoul strict_strtoul
#define kstrtol strict_strtol
#endif

#endif /* COMPAT_H */
