/*
 * Copyright (C) 2025, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Dual:>>
 */

#ifndef _TYPEDEFS_H_
#define _TYPEDEFS_H_

#if (!defined(EDK_RELEASE_VERSION) || (EDK_RELEASE_VERSION < 0x00020000)) || \
	!defined(BWL_NO_INTERNAL_STDLIB_SUPPORT)

#ifdef SITE_TYPEDEFS

/*
 * Define SITE_TYPEDEFS in the compile to include a site-specific
 * typedef file "site_typedefs.h".
 *
 * If SITE_TYPEDEFS is not defined, then the code section below makes
 * inferences about the compile environment based on defined symbols and
 * possibly compiler pragmas.
 *
 * Following these two sections is the Default Typedefs section.
 * This section is only processed if USE_TYPEDEF_DEFAULTS is
 * defined. This section has a default set of typedefs and a few
 * preprocessor symbols (TRUE, FALSE, NULL, ...).
 */

#include "site_typedefs.h"

#else

/*
 * Infer the compile environment based on preprocessor symbols and pragmas.
 * Override type definitions as needed, and include configuration-dependent
 * header files to define types.
 */

#ifdef __cplusplus

#define TYPEDEF_BOOL
#ifndef FALSE
#define FALSE	false
#endif
#ifndef TRUE
#define TRUE	true
#endif

#else	/* ! __cplusplus */

#if defined(_WIN32)

#define TYPEDEF_BOOL
typedef	unsigned char	bool;			/* consistent w/BOOL */

#endif /* _WIN32 */

#endif	/* ! __cplusplus */

#if defined(EFI) && !defined(EFI_WINBLD) && !defined(__size_t__)
typedef long unsigned int size_t;
#endif /* EFI */

#if !defined(TYPEDEF_UINTPTR)
#if defined(_WIN64) && !defined(EFI)
/* use the Windows ULONG_PTR type when compiling for 64 bit */
#include <basetsd.h>
#define TYPEDEF_UINTPTR
typedef ULONG_PTR uintptr;
#elif defined(__LP64__)
#define TYPEDEF_UINTPTR
typedef unsigned long long int uintptr;
#endif
#endif /* TYPEDEF_UINTPTR */

#if defined(_RTE_) || defined(COEX_CPU_BUILD)
#define _NEED_SIZE_T_
#endif

/* float_t types conflict with the same typedefs from the standard ANSI-C
** math.h header file. Don't re-typedef them here.
*/

#if defined(MACOSX) || defined(__linux__)
#define TYPEDEF_FLOAT_T
#endif /* MACOSX || Linux */

#if defined(_NEED_SIZE_T_)
typedef long unsigned int size_t;
#endif

#ifdef _MSC_VER	/* Microsoft C */
#define TYPEDEF_INT64
#define TYPEDEF_UINT64
typedef signed __int64	int64;
typedef unsigned __int64 uint64;
#endif

#if defined(__sparc__)
#define TYPEDEF_ULONG
#endif

#if defined(__linux__) && !defined(EFI)
/*
 * If this is either a Linux hybrid build or the per-port code of a hybrid build
 * then use the Linux header files to get some of the typedefs.  Otherwise, define
 * them entirely in this file.  We can't always define the types because we get
 * a duplicate typedef error; there is no way to "undefine" a typedef.
 * We know when it's per-port code because each file defines LINUX_PORT at the top.
 */
#define TYPEDEF_UINT
#ifndef TARGETENV_android
#define TYPEDEF_USHORT
#define TYPEDEF_ULONG
#endif /* TARGETENV_android */
#ifdef __KERNEL__
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19))
#define TYPEDEF_BOOL
#endif	/* >= 2.6.19 */
/* special detection for 2.6.18-128.7.1.0.1.el5 */
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18))
#include <linux/compiler.h>
#ifdef noinline_for_stack
#define TYPEDEF_BOOL
#endif
#endif	/* == 2.6.18 */
#endif	/* __KERNEL__ */
#endif	/* linux && !EFI */

#if !defined(__linux__) && !defined(_WIN32) && !defined(_RTE_) && !defined(__DJGPP__) \
	&& !defined(__BOB__) && !defined(EFI) && !defined(COEX_CPU_BUILD)
#define TYPEDEF_UINT
#define TYPEDEF_USHORT
#endif

/* Do not support the (u)int64 types with strict ansi for GNU C */
#if defined(__GNUC__) && defined(__STRICT_ANSI__) && (__STDC_VERSION__ < 199901L)
#define TYPEDEF_INT64
#define TYPEDEF_UINT64
#endif /* defined(__GNUC__) && defined(__STRICT_ANSI__) */

/* ICL accepts unsigned 64 bit type only, and complains in ANSI mode
 * for signed or unsigned
 */
#if defined(__ICL)

#define TYPEDEF_INT64

#if defined(__STDC__) && (__STDC_VERSION__ < 199901L)
#define TYPEDEF_UINT64
#endif

#endif /* __ICL */

#if !defined(_WIN32) && !defined(_RTE_) && !defined(COEX_CPU_BUILD) && \
	!defined(__DJGPP__) && !defined(__BOB__) && !defined(EFI)

/* pick up ushort & uint from standard types.h */
#if defined(__linux__) && defined(__KERNEL__)

/* See note above */
#ifdef USER_MODE
#include <sys/types.h>
#else
#include <linux/types.h>	/* sys/types.h and linux/types.h are oil and water */
#endif /* USER_MODE */

#else

#include <sys/types.h>

#endif /* linux && __KERNEL__ */

#endif /* !_WIN32 && !_RTE_  && !__DJGPP__ */

/* use the default typedefs in the next section of this file */
#define USE_TYPEDEF_DEFAULTS

#endif /* SITE_TYPEDEFS */

/*
 * Default Typedefs
 */

#ifdef USE_TYPEDEF_DEFAULTS
#undef USE_TYPEDEF_DEFAULTS

#ifndef TYPEDEF_BOOL
typedef	/* @abstract@ */ unsigned char	bool;
#endif /* endif TYPEDEF_BOOL */

/* define uchar, ushort, uint, ulong */

#ifndef TYPEDEF_UCHAR
typedef unsigned char	uchar;
#endif

#ifndef TYPEDEF_USHORT
typedef unsigned short	ushort;
#endif

#ifndef TYPEDEF_UINT
typedef unsigned int	uint;
#endif

#ifndef TYPEDEF_ULONG
typedef unsigned long	ulong;
#endif

/* define [u]int8/16/32/64, uintptr */

#ifndef TYPEDEF_UINT8
typedef unsigned char	uint8;
#endif

#ifndef TYPEDEF_UINT16
typedef unsigned short	uint16;
#endif

#ifndef TYPEDEF_UINT32
typedef unsigned int	uint32;
#endif

#ifndef TYPEDEF_UINT64
typedef unsigned long long uint64;
#endif

#ifndef TYPEDEF_UINTPTR
typedef unsigned int	uintptr;
#endif

#ifndef TYPEDEF_INT8
typedef signed char	int8;
#endif

#ifndef TYPEDEF_INT16
typedef signed short	int16;
#endif

#ifndef TYPEDEF_INT32
typedef signed int	int32;
#endif

#ifndef TYPEDEF_INT64
typedef signed long long int64;
#endif

/* define float32/64, float_t */

#ifndef TYPEDEF_FLOAT32
typedef float		float32;
#endif

#ifndef TYPEDEF_FLOAT64
typedef double		float64;
#endif

/*
 * abstracted floating point type allows for compile time selection of
 * single or double precision arithmetic.  Compiling with -DFLOAT32
 * selects single precision; the default is double precision.
 */

#ifndef TYPEDEF_FLOAT_T

#if defined(FLOAT32)
typedef float32 float_t;
#else /* default to double precision floating point */
typedef float64 float_t;
#endif

#endif /* TYPEDEF_FLOAT_T */

/* define macro values */

#ifndef FALSE
#define FALSE	0
#endif

#ifndef TRUE
#define TRUE	1  /* TRUE */
#endif

#ifndef NULL
#define	NULL	0
#endif

#ifndef OFF
#define	OFF	0
#endif

#ifndef ON
#define	ON	1  /* ON = 1 */
#endif

#define	AUTO	(-1) /* Auto = -1 */

/* define PTRSZ, INLINE */

#ifndef PTRSZ
#define	PTRSZ	sizeof(char*)
#endif

/* Detect compiler type. */
#ifdef _MSC_VER
	#define BWL_COMPILER_MICROSOFT
#elif defined(__GNUC__) || defined(__lint)
	#define BWL_COMPILER_GNU
#elif defined(__CC_ARM) && __CC_ARM
	#define BWL_COMPILER_ARMCC
#else
	#error "Unknown compiler!"
#endif /* _MSC_VER */

#ifndef INLINE
#if defined(BWL_COMPILER_MICROSOFT)
	#define INLINE __inline
#elif defined(BWL_COMPILER_GNU)
	#define INLINE __inline__
#elif defined(BWL_COMPILER_ARMCC)
	#define INLINE	__inline
#else
	#define INLINE
#endif /* _MSC_VER */
#endif /* INLINE */

/* Force inlining. */
#if defined(BWL_COMPILER_GNU)
#define INLINE_ALWAYS	inline  __attribute__ ((always_inline))
#else
#define INLINE_ALWAYS	INLINE
#endif

#undef TYPEDEF_BOOL
#undef TYPEDEF_UCHAR
#undef TYPEDEF_USHORT
#undef TYPEDEF_UINT
#undef TYPEDEF_ULONG
#undef TYPEDEF_UINT8
#undef TYPEDEF_UINT16
#undef TYPEDEF_UINT32
#undef TYPEDEF_UINT64
#undef TYPEDEF_UINTPTR
#undef TYPEDEF_INT8
#undef TYPEDEF_INT16
#undef TYPEDEF_INT32
#undef TYPEDEF_INT64
#undef TYPEDEF_FLOAT32
#undef TYPEDEF_FLOAT64
#undef TYPEDEF_FLOAT_T

#endif /* USE_TYPEDEF_DEFAULTS */

/* Suppress unused parameter warning */
#define UNUSED_PARAMETER(x) (void)(x)

/* Avoid warning for discarded const or volatile qualifier in special cases (-Wcast-qual) */
#define DISCARD_QUAL(ptr, type) ((type *)(uintptr)(ptr))

#else /* !EDK_RELEASE_VERSION || (EDK_RELEASE_VERSION < 0x00020000) */

#include <sys/types.h>
#include <strings.h>
#include <stdlib.h>

#ifdef stderr
#undef stderr
#define stderr stdout
#endif

typedef UINT8   uint8;
typedef UINT16  uint16;
typedef UINT32  uint32;
typedef UINT64  uint64;
typedef INT8    int8;
typedef INT16   int16;
typedef INT32   int32;
typedef INT64   int64;

typedef BOOLEAN       bool;
typedef unsigned char uchar;
typedef UINTN         uintptr;

#define UNUSED_PARAMETER(x) (void)(x)
#define DISCARD_QUAL(ptr, type) ((type *)(uintptr)(ptr))
#define INLINE
#define INLINE_ALWAYS	INLINE
#define	AUTO	(-1) /* Auto = -1 */
#define	ON	1  /* ON = 1 */
#define	OFF	0

#endif /* !EDK_RELEASE_VERSION || (EDK_RELEASE_VERSION < 0x00020000) */

/*
 * Including the bcmdefs.h here, to make sure everyone including typedefs.h
 * gets this automatically
*/
#include <bcmdefs.h>
#endif /* _TYPEDEFS_H_ */
