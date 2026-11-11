/*
 * kernel_xbox.c - Xbox Identity & Hardware Stubs
 *
 * Provides Xbox-specific exported data (hardware info, kernel version,
 * keys, image filename) and section loading stubs.
 *
 * Most Xbox-specific hardware data is stubbed with plausible retail
 * values. Crypto keys are zeroed since we don't need Xbox Live or
 * EEPROM-based encryption on PC.
 */

#include "kernel.h"
#include "kernel_language.h"
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#if defined(_WIN32)
#  define strcasecmp _stricmp
#else
#  include <strings.h>
#endif

/* ============================================================================
 * Exported Data Objects
 *
 * These are global variables exported by the Xbox kernel at known ordinals.
 * Game code accesses them directly via the thunk table.
 * ============================================================================ */

/* Hardware info - default report is a standard retail Xbox.
 * A target may override via xbox_kernel_set_version() or by patching the
 * global directly; no title-specific values are baked in. */
XBOX_HARDWARE_INFO xbox_HardwareInfo = {
    .Flags       = 0x00000020,  /* Retail Xbox */
    .GpuRevision = 0xD2,       /* NV2A D2 revision */
    .McpRevision = 0xD4,       /* MCPX D4 revision */
    .Reserved    = {0, 0}
};

/* Kernel version - default is a stock retail value. Titles built against a
 * different XDK can report their own through xbox_kernel_set_version(). */
XBOX_KRNL_VERSION xbox_KrnlVersion = {
    .Major = 1,
    .Minor = 0,
    .Build = 5849,
    .Qfe   = 1
};

/* Override the kernel version reported to game code (ordinal 324).
 * Match the XDK revision the target title was built with if it checks it. */
void xbox_kernel_set_version(USHORT major, USHORT minor, USHORT build, USHORT qfe)
{
    xbox_KrnlVersion.Major = major;
    xbox_KrnlVersion.Minor = minor;
    xbox_KrnlVersion.Build = build;
    xbox_KrnlVersion.Qfe   = qfe;
}

/* Crypto keys - zeroed, not needed for PC operation.
 * These are unique per-console on real hardware. */
UCHAR xbox_EEPROMKey[16]              = {0};
UCHAR xbox_HDKey[16]                  = {0};
UCHAR xbox_SignatureKey[16]           = {0};
UCHAR xbox_LANKey[16]                 = {0};
UCHAR xbox_AlternateSignatureKeys[16][16] = {{0}};

/* A title signs its saves with these keys (XCalculateSignature mixes the
 * signature key with the title's; "noroam" saves add the HD key), so a save
 * moves between hosts only when they match. Every build leaves them zero;
 * the saves zip records which set made it (openspec overlay-saves, D1). */
const char *xbox_saves_keys_id(void)
{
    static char id[17];
    static const char hex[] = "0123456789abcdef";
    XBOX_SHA_CONTEXT sha;
    UCHAR d[20];
    int i, zero = 1;
    for (i = 0; i < 16; i++)
        if (xbox_HDKey[i] || xbox_SignatureKey[i])
            zero = 0;
    if (zero)
        return "zero";
    xbox_XcSHAInit(&sha);
    xbox_XcSHAUpdate(&sha, (const UCHAR *)"xbr-saves-keys", 14);
    xbox_XcSHAUpdate(&sha, xbox_HDKey, 16);
    xbox_XcSHAUpdate(&sha, xbox_SignatureKey, 16);
    xbox_XcSHAFinal(&sha, d);
    for (i = 0; i < 8; i++) {
        id[2 * i] = hex[d[i] >> 4];
        id[2 * i + 1] = hex[d[i] & 15];
    }
    id[16] = '\0';
    return id;
}

/* Public key data for Xbox Live signature verification - not needed */
UCHAR xbox_XePublicKeyData[284] = {0};

/* Image filename - the XBE path as seen by the kernel */
static char g_image_filename[] = "\\Device\\CdRom0\\default.xbe";
XBOX_ANSI_STRING xbox_XeImageFileName = {
    .Length        = sizeof(g_image_filename) - 1,
    .MaximumLength = sizeof(g_image_filename),
    .Buffer        = g_image_filename
};

/* Launch data page - used for title-to-title launches (e.g., Xbox Dashboard → game).
 * Allocated dynamically and zeroed for normal game boot. */
static XBOX_LAUNCH_DATA_PAGE g_launch_data_page = {0};
XBOX_LAUNCH_DATA_PAGE* xbox_LaunchDataPage = &g_launch_data_page;

/* ============================================================================
 * Section Loading
 *
 * XeLoadSection/XeUnloadSection manage on-demand loading of XBE sections.
 * On Xbox, some sections are demand-paged from disc. In our recompilation,
 * the entire executable is loaded into memory, so these are reference-counting
 * no-ops. Recompiled code never reaches them: ordinals 327/328 go through
 * bridge_XeSection (kernel_bridge.c), which is where section paging
 * (xbe_sections.c) lives -- change that path, not this one.
 * ============================================================================ */

NTSTATUS __stdcall xbox_XeLoadSection(PXBE_SECTION_HEADER Section)
{
    if (!Section)
        return STATUS_INVALID_PARAMETER;

    InterlockedIncrement(&Section->SectionReferenceCount);

    xbox_log(XBOX_LOG_DEBUG, XBOX_LOG_XBOX,
        "XeLoadSection: '%s' at %p (size=%u, refcount=%d)",
        Section->SectionName ? Section->SectionName : "<null>",
        Section->VirtualAddress, Section->VirtualSize,
        Section->SectionReferenceCount);

    return STATUS_SUCCESS;
}

NTSTATUS __stdcall xbox_XeUnloadSection(PXBE_SECTION_HEADER Section)
{
    if (!Section)
        return STATUS_INVALID_PARAMETER;

    LONG new_count = InterlockedDecrement(&Section->SectionReferenceCount);

    xbox_log(XBOX_LOG_DEBUG, XBOX_LOG_XBOX,
        "XeUnloadSection: '%s' (refcount=%d)",
        Section->SectionName ? Section->SectionName : "<null>",
        new_count);

    return STATUS_SUCCESS;
}

/* ============================================================================
 * EEPROM / Non-Volatile Settings
 *
 * ExQueryNonVolatileSetting / ExSaveNonVolatileSetting read and write EEPROM
 * settings. On real hardware these are stored in the 256-byte EEPROM on the
 * SMBus. For recompilation, we return sensible defaults:
 *   - Region: North America
 *   - Video: NTSC, widescreen+HDTV enabled
 *   - Language: system.language, English by default (kernel_language.h)
 *   - Audio: Stereo, Dolby Digital
 *   - DVD Region: Region 1 (North America)
 *   - Time zone: the host's
 * ============================================================================ */

/* The console's time zone. The EEPROM holds the bias (UTC = local + bias,
 * in minutes), the standard and daylight biases added to it, four-character
 * names, and the dates daylight time starts and ends (month, week of the
 * month, day of the week, hour). XAPI turns the UTC clock into local time
 * with these, so a fixed zone stamps saves and clocks hours off anywhere
 * else: SSX 3 dated its saves two hours early in CEST.
 *
 * Windows has all of it (GetTimeZoneInformation). POSIX hosts give the
 * offset in effect now and the zone names, not the rule: the bias is the
 * current offset with no daylight dates, which is right until the next
 * change of clocks during a run. */
typedef struct { uint8_t month, day, dow, hour; } XboxTzDate;
static struct {
    int32_t bias, std_bias, dlt_bias;
    XboxTzDate std_date, dlt_date;
    char std_name[4], dlt_name[4];
} s_tz;

static void tz_name4(char out[4], const char *src)
{
    int k;
    for (k = 0; k < 4; k++) {
        char c = (src && *src) ? *src++ : 0;
        out[k] = (c >= 0x20 && c < 0x7F) ? c : (c ? '?' : 0);
    }
}

static void tz_init(void)
{
    static int done;
    if (done)
        return;
    done = 1;
#ifdef _WIN32
    {
        TIME_ZONE_INFORMATION tzi;
        char sn[5], dn[5];
        int k;
        memset(&tzi, 0, sizeof tzi);
        GetTimeZoneInformation(&tzi);
        s_tz.bias = tzi.Bias;
        s_tz.std_bias = tzi.StandardBias;
        s_tz.dlt_bias = tzi.DaylightBias;
        s_tz.std_date.month = (uint8_t)tzi.StandardDate.wMonth;
        s_tz.std_date.day = (uint8_t)tzi.StandardDate.wDay;
        s_tz.std_date.dow = (uint8_t)tzi.StandardDate.wDayOfWeek;
        s_tz.std_date.hour = (uint8_t)tzi.StandardDate.wHour;
        s_tz.dlt_date.month = (uint8_t)tzi.DaylightDate.wMonth;
        s_tz.dlt_date.day = (uint8_t)tzi.DaylightDate.wDay;
        s_tz.dlt_date.dow = (uint8_t)tzi.DaylightDate.wDayOfWeek;
        s_tz.dlt_date.hour = (uint8_t)tzi.DaylightDate.wHour;
        for (k = 0; k < 4; k++) {
            sn[k] = (char)(tzi.StandardName[k] < 128 ? tzi.StandardName[k] : '?');
            dn[k] = (char)(tzi.DaylightName[k] < 128 ? tzi.DaylightName[k] : '?');
        }
        sn[4] = dn[4] = 0;
        tz_name4(s_tz.std_name, sn);
        tz_name4(s_tz.dlt_name, dn);
    }
#else
    {
        time_t now = time(NULL);
        struct tm lt;
        tzset();
        if (localtime_r(&now, &lt))
            s_tz.bias = (int32_t)(-lt.tm_gmtoff / 60);
        tz_name4(s_tz.std_name, tzname[0]);
        tz_name4(s_tz.dlt_name, tzname[1]);
    }
#endif
}

/* XDK's XC_VIDEO_FLAGS_60Hz (the user's "PAL 60" choice); this branch's
 * kernel.h keeps the other flags in the low byte. */
#ifndef XC_VIDEO_FLAGS_60Hz
#define XC_VIDEO_FLAGS_60Hz 0x00400000
#endif

/* RECOMP_VIDEO_STANDARD / RECOMP_AV_PACK: which console this is.
 *
 * Everything here used to say "North American NTSC console", and the factory
 * block was not answered at all, so XGetVideoStandard() -- byte 1 of
 * XC_FACTORY_AV_REGION -- returned 0, which is no standard. A PAL release
 * can depend on the answer for more than its display mode: Enter the Matrix
 * (Europe) appends "pal" to every movie name when the standard is PAL-I, its
 * disc holds only the .vidpal files, and on an NTSC console it stops at
 * "There's a problem with the disc you're using". */
const XboxVideoStandard *xbox_video_standard(void)
{
    static XboxVideoStandard vs;
    static int done;
    const char *e, *p;

    if (done)
        return &vs;
    e = getenv("RECOMP_VIDEO_STANDARD");
    vs.av_standard = AV_STANDARD_NTSC_M;
    vs.av_region = XC_AV_REGION_NTSC_M;
    vs.game_region = XC_GAME_REGION_NA;
    vs.dvd_region = 1;
    vs.video_flags = XC_VIDEO_FLAGS_60Hz;
    vs.av_refresh = AV_REFRESH_60Hz;
    vs.av_pack = AV_PACK_HDTV;
    if (e && (!strcasecmp(e, "pal") || !strcasecmp(e, "pal-i")
              || !strcasecmp(e, "pal50") || !strcasecmp(e, "pal60"))) {
        int pal60 = !strcasecmp(e, "pal60");
        vs.av_standard = AV_STANDARD_PAL_I;
        vs.av_region = XC_AV_REGION_PAL_I;
        vs.game_region = XC_GAME_REGION_RESTOFWORLD;
        vs.dvd_region = 2;
        vs.video_flags = pal60 ? XC_VIDEO_FLAGS_60Hz : 0;
        vs.av_refresh = pal60 ? AV_REFRESH_60Hz : AV_REFRESH_50Hz;
        vs.av_pack = AV_PACK_SCART;
    } else if (e && (!strcasecmp(e, "ntsc-j") || !strcasecmp(e, "ntscj"))) {
        vs.av_standard = AV_STANDARD_NTSC_J;
        vs.av_region = XC_AV_REGION_NTSC_J;
        vs.game_region = XC_GAME_REGION_JAPAN;
        vs.dvd_region = 2;
    } else if (e && *e && strcasecmp(e, "ntsc") && strcasecmp(e, "ntsc-m")) {
        fprintf(stderr, "RECOMP_VIDEO_STANDARD=%s not recognised "
                        "(ntsc-m, ntsc-j, pal, pal60); using ntsc-m\n", e);
    }
    p = getenv("RECOMP_AV_PACK");
    if (p && *p) {
        static const struct { const char *name; uint32_t pack; } packs[] = {
            { "standard", AV_PACK_STANDARD }, { "rfu", AV_PACK_RFU },
            { "scart", AV_PACK_SCART }, { "hdtv", AV_PACK_HDTV },
            { "vga", AV_PACK_VGA }, { "svideo", AV_PACK_SVIDEO },
        };
        size_t k;
        for (k = 0; k < sizeof packs / sizeof packs[0]; k++)
            if (!strcasecmp(p, packs[k].name))
                vs.av_pack = packs[k].pack;
    }
    done = 1;
    return &vs;
}

NTSTATUS __stdcall xbox_ExQueryNonVolatileSetting(
    ULONG ValueIndex, PULONG Type, PVOID Value, ULONG ValueLength, PULONG ResultLength)
{
    if (!Value)
        return STATUS_INVALID_PARAMETER;
    tz_init();

    xbox_log(XBOX_LOG_DEBUG, XBOX_LOG_XBOX,
        "ExQueryNonVolatileSetting: index=0x%02X len=%u", ValueIndex, ValueLength);

    switch (ValueIndex) {
    case XC_LANGUAGE:
        /* system.language; English (1) unless set (kernel_language.h). */
        if (ValueLength >= sizeof(ULONG)) {
            *(PULONG)Value = xbox_Language();
            if (Type) *Type = 4; /* REG_DWORD */
            if (ResultLength) *ResultLength = sizeof(ULONG);
        }
        break;

    case XC_VIDEO:
        /* Widescreen and HDTV, plus the TV system's own flags (PAL-I, 50/60 Hz:
         * RECOMP_VIDEO_STANDARD). */
        if (ValueLength >= sizeof(ULONG)) {
            *(PULONG)Value = XC_VIDEO_FLAGS_WIDESCREEN | XC_VIDEO_FLAGS_HDTV
                           | xbox_video_standard()->video_flags;
            if (Type) *Type = 4; /* REG_DWORD */
            if (ResultLength) *ResultLength = sizeof(ULONG);
        }
        break;

    case XC_AUDIO:
        /* Stereo PCM matches the two-channel host output. 1 means mono;
         * advertising AC3 requires an encoded-output path. */
        if (ValueLength >= sizeof(ULONG)) {
            *(PULONG)Value = 0x00000000;
            if (Type) *Type = 4; /* REG_DWORD */
            if (ResultLength) *ResultLength = sizeof(ULONG);
        }
        break;

    case XC_P_CONTROL_GAMES:
    case XC_P_CONTROL_MOVIES:
        /* Unrestricted. Titles compare their XBE certificate's rating against
         * this and refuse to run (Halo boots to the dashboard) if the console
         * is configured more strictly, so 0 means "no restriction". */
        if (ValueLength >= sizeof(ULONG)) {
            *(PULONG)Value = 0;
            if (Type) *Type = 4;
            if (ResultLength) *ResultLength = sizeof(ULONG);
        }
        break;

    case XC_DVD_REGION:
        /* Region 1 (North America) unless RECOMP_VIDEO_STANDARD says otherwise */
        if (ValueLength >= sizeof(ULONG)) {
            *(PULONG)Value = xbox_video_standard()->dvd_region;
            if (Type) *Type = 4;
            if (ResultLength) *ResultLength = sizeof(ULONG);
        }
        break;

    case XC_FACTORY_AV_REGION:
    case XC_FACTORY_GAME_REGION:
        if (ValueLength >= sizeof(ULONG)) {
            *(PULONG)Value = ValueIndex == XC_FACTORY_AV_REGION
                           ? xbox_video_standard()->av_region
                           : xbox_video_standard()->game_region;
            if (Type) *Type = 4;
            if (ResultLength) *ResultLength = sizeof(ULONG);
        }
        break;

    case XC_MISC:
        /* Misc flags: 0 = no special flags */
        if (ValueLength >= sizeof(ULONG)) {
            *(PULONG)Value = 0;
            if (Type) *Type = 4;
            if (ResultLength) *ResultLength = sizeof(ULONG);
        }
        break;

    case XC_TIMEZONE_BIAS:
    case XC_TZ_STD_BIAS:
    case XC_TZ_DLT_BIAS:
        if (ValueLength >= sizeof(LONG)) {
            *(PLONG)Value = ValueIndex == XC_TIMEZONE_BIAS ? s_tz.bias
                          : ValueIndex == XC_TZ_STD_BIAS ? s_tz.std_bias : s_tz.dlt_bias;
            if (Type) *Type = 4;
            if (ResultLength) *ResultLength = sizeof(LONG);
        }
        break;

    case XC_TZ_STD_NAME:
    case XC_TZ_DLT_NAME:
        if (ValueLength >= 4) {
            memcpy(Value, ValueIndex == XC_TZ_STD_NAME ? s_tz.std_name : s_tz.dlt_name, 4);
            if (Type) *Type = 3;                 /* REG_BINARY */
            if (ResultLength) *ResultLength = 4;
        }
        break;

    case XC_TZ_STD_DATE:
    case XC_TZ_DLT_DATE:
        if (ValueLength >= 4) {
            memcpy(Value, ValueIndex == XC_TZ_STD_DATE ? &s_tz.std_date : &s_tz.dlt_date, 4);
            if (Type) *Type = 4;
            if (ResultLength) *ResultLength = 4;
        }
        break;

    case XC_MAX_OS: {
        /* The whole user-settings block (EEPROM 0x60..), which XAPI reads in
         * one go: checksum, bias, std/dlt names, 8 reserved, std/dlt dates,
         * 8 reserved, std/dlt bias, language, video, audio, parental x3,
         * online x4, misc, DVD region -- the same values as the single
         * queries above. Answering it with zeroes put every title in UTC. */
        uint8_t blk[0x60];
        memset(blk, 0, sizeof blk);
        memcpy(blk + 0x04, &s_tz.bias, 4);
        memcpy(blk + 0x08, s_tz.std_name, 4);
        memcpy(blk + 0x0C, s_tz.dlt_name, 4);
        memcpy(blk + 0x18, &s_tz.std_date, 4);
        memcpy(blk + 0x1C, &s_tz.dlt_date, 4);
        memcpy(blk + 0x28, &s_tz.std_bias, 4);
        memcpy(blk + 0x2C, &s_tz.dlt_bias, 4);
        { ULONG v = xbox_Language(); memcpy(blk + 0x30, &v, 4); }         /* XC_LANGUAGE */
        { ULONG v = XC_VIDEO_FLAGS_WIDESCREEN | XC_VIDEO_FLAGS_HDTV
                  | xbox_video_standard()->video_flags; memcpy(blk + 0x34, &v, 4); }
        /* 0x58 is MiscFlags (0); DvdRegion is the last field, at 0x5C. */
        { ULONG v = xbox_video_standard()->dvd_region; memcpy(blk + 0x5C, &v, 4); }
        if (Type) *Type = 3;
        if (ResultLength) *ResultLength = sizeof blk;
        if (ValueLength < sizeof blk)
            return 0xC0000023L;                       /* STATUS_BUFFER_TOO_SMALL */
        memcpy(Value, blk, sizeof blk);
        if (ValueLength > sizeof blk)
            memset((uint8_t *)Value + sizeof blk, 0, ValueLength - sizeof blk);
        break;
    }

    default:
        xbox_log(XBOX_LOG_WARN, XBOX_LOG_XBOX,
            "ExQueryNonVolatileSetting: unhandled index 0x%02X", ValueIndex);
        memset(Value, 0, ValueLength);
        if (Type) *Type = 4;
        if (ResultLength) *ResultLength = ValueLength;
        break;
    }

    return STATUS_SUCCESS;
}

NTSTATUS __stdcall xbox_ExSaveNonVolatileSetting(
    ULONG ValueIndex, ULONG Type, PVOID Value, ULONG ValueLength)
{
    (void)Type;
    (void)Value;
    (void)ValueLength;

    xbox_log(XBOX_LOG_INFO, XBOX_LOG_XBOX,
        "ExSaveNonVolatileSetting: index=0x%02X len=%u (ignored - read-only on PC)",
        ValueIndex, ValueLength);

    /* Accept writes silently but don't persist them */
    return STATUS_SUCCESS;
}

/* ============================================================================
 * Network / PHY
 *
 * PhyGetLinkState reports Ethernet link status. Xbox had a built-in 100Mbit
 * NIC. For PC, we report link-up since we'll handle networking differently.
 * ============================================================================ */

ULONG __stdcall xbox_PhyGetLinkState(BOOLEAN Verify)
{
    (void)Verify;
    /* Return link up, 100 Mbps, full duplex */
    return 0x01; /* XNET_ETHERNET_LINK_ACTIVE */
}

NTSTATUS __stdcall xbox_PhyInitialize(BOOLEAN ForceReset, PVOID Param2)
{
    (void)ForceReset;
    (void)Param2;
    return STATUS_SUCCESS;
}
