#include <stddef.h>
#include <stdint.h>
#include <sys/sndstat.h>
#include <sys/soundcard.h>

struct pw_oss_abi_entry {
	const char *name;
	uint64_t value;
};

#define ABI_SIZE(name, type) { "size." name, (uint64_t)sizeof(type) }
#define ABI_ALIGN(name, type) { "align." name, (uint64_t)_Alignof(type) }
#define ABI_OFFSET(name, type, field) \
	{ "offset." name "." #field, (uint64_t)offsetof(type, field) }
#define ABI_CONST(name) { "const." #name, (uint64_t)(name) }

static const struct pw_oss_abi_entry pw_oss_abi_report[] = {
	ABI_SIZE("SndstiocNvArg", struct sndstioc_nv_arg),
	ABI_ALIGN("SndstiocNvArg", struct sndstioc_nv_arg),
	ABI_OFFSET("SndstiocNvArg", struct sndstioc_nv_arg, nbytes),
	ABI_OFFSET("SndstiocNvArg", struct sndstioc_nv_arg, buf),

	ABI_SIZE("audio_buf_info", audio_buf_info),
	ABI_ALIGN("audio_buf_info", audio_buf_info),
	ABI_OFFSET("audio_buf_info", audio_buf_info, fragments),
	ABI_OFFSET("audio_buf_info", audio_buf_info, fragstotal),
	ABI_OFFSET("audio_buf_info", audio_buf_info, fragsize),
	ABI_OFFSET("audio_buf_info", audio_buf_info, bytes),

	ABI_SIZE("audio_errinfo", audio_errinfo),
	ABI_ALIGN("audio_errinfo", audio_errinfo),
	ABI_OFFSET("audio_errinfo", audio_errinfo, play_underruns),
	ABI_OFFSET("audio_errinfo", audio_errinfo, rec_overruns),
	ABI_OFFSET("audio_errinfo", audio_errinfo, play_ptradjust),
	ABI_OFFSET("audio_errinfo", audio_errinfo, rec_ptradjust),
	ABI_OFFSET("audio_errinfo", audio_errinfo, play_errorcount),
	ABI_OFFSET("audio_errinfo", audio_errinfo, rec_errorcount),
	ABI_OFFSET("audio_errinfo", audio_errinfo, play_lasterror),
	ABI_OFFSET("audio_errinfo", audio_errinfo, rec_lasterror),
	ABI_OFFSET("audio_errinfo", audio_errinfo, play_errorparm),
	ABI_OFFSET("audio_errinfo", audio_errinfo, rec_errorparm),
	ABI_OFFSET("audio_errinfo", audio_errinfo, filler),

	ABI_SIZE("oss_audioinfo", oss_audioinfo),
	ABI_ALIGN("oss_audioinfo", oss_audioinfo),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, dev),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, name),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, busy),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, pid),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, caps),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, iformats),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, oformats),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, magic),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, cmd),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, card_number),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, port_number),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, mixer_dev),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, legacy_device),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, enabled),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, flags),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, min_rate),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, max_rate),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, min_channels),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, max_channels),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, binding),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, rate_source),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, handle),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, nrates),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, rates),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, song_name),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, label),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, latency),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, devnode),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, next_play_engine),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, next_rec_engine),
	ABI_OFFSET("oss_audioinfo", oss_audioinfo, filler),

	ABI_SIZE("OssChannelOrder", unsigned long long),
	ABI_ALIGN("OssChannelOrder", unsigned long long),

	ABI_CONST(AFMT_U8),
	ABI_CONST(AFMT_S16_LE),
	ABI_CONST(AFMT_S16_BE),
	ABI_CONST(AFMT_S32_LE),
	ABI_CONST(AFMT_S32_BE),
	ABI_CONST(AFMT_S24_LE),
	ABI_CONST(AFMT_S24_BE),
	ABI_CONST(AFMT_F32_LE),
	ABI_CONST(AFMT_F32_BE),
	ABI_CONST(PCM_ENABLE_INPUT),
	ABI_CONST(PCM_ENABLE_OUTPUT),
	ABI_CONST(PCM_CAP_INPUT),
	ABI_CONST(PCM_CAP_OUTPUT),
	ABI_CONST(PCM_CAP_VIRTUAL),
	ABI_CONST(SNDCTL_DSP_SPEED),
	ABI_CONST(SNDCTL_DSP_SETFMT),
	ABI_CONST(SNDCTL_DSP_CHANNELS),
	ABI_CONST(SNDCTL_DSP_SETFRAGMENT),
	ABI_CONST(SNDCTL_DSP_LOW_WATER),
	ABI_CONST(SNDCTL_DSP_GETFMTS),
	ABI_CONST(SNDCTL_DSP_GETOSPACE),
	ABI_CONST(SNDCTL_DSP_GETISPACE),
	ABI_CONST(SNDCTL_DSP_SETTRIGGER),
	ABI_CONST(SNDCTL_DSP_GETODELAY),
	ABI_CONST(SNDCTL_DSP_GETERROR),
	ABI_CONST(SNDCTL_DSP_GET_CHNORDER),
	ABI_CONST(SNDCTL_DSP_SET_CHNORDER),
	ABI_CONST(SNDCTL_DSP_HALT),
	ABI_CONST(SNDCTL_DSP_SILENCE),
	ABI_CONST(SNDCTL_DSP_SKIP),
	ABI_CONST(SNDCTL_ENGINEINFO),
	ABI_CONST(SNDSTIOC_REFRESH_DEVS),
	ABI_CONST(SNDSTIOC_GET_DEVS),
};

const struct pw_oss_abi_entry *
pw_oss_native_abi_report(size_t *count)
{
	*count = sizeof(pw_oss_abi_report) / sizeof(pw_oss_abi_report[0]);
	return (pw_oss_abi_report);
}
