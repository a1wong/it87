/*---------------------------------------------------------------------------
 * 
 * compat.h 
 *     Copyright (c) 2012 Guenter Roeck <linux@roeck-us.net>
 *
 *---------------------------------------------------------------------------
 */

#ifndef COMPAT_H
#define COMPAT_H

#ifndef kstrtol
#define kstrtol strict_strtol
#endif
#ifndef kstrtoul
#define kstrtoul strict_strtoul
#endif

#ifndef request_muxed_region
#define request_muxed_region(a, b, c) (true)
#define release_region(a, b)
#endif

#endif /* COMPAT_H */
