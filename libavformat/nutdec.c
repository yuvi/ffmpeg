/*
 * "NUT" Container Format demuxer
 * Copyright (c) 2004-2006 Michael Niedermayer
 * Copyright (c) 2003 Alex Beregszaszi
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#include "nut.h"

#undef NDEBUG
#include <assert.h>

static uint64_t get_v(ByteIOContext *bc/*, maxstuffing*/){
    uint64_t val = 0;

    for(;;)
    {
        int tmp = get_byte(bc);

//         if(tmp=0x80){
//             if(!maxstuffing-- || val)
//                 return -1;
//         }

        if (tmp&0x80)
            val= (val<<7) + tmp - 0x80;
        else{
            return (val<<7) + tmp;
        }
    }
    return -1;
}

static int get_str(ByteIOContext *bc, char *string, unsigned int maxlen){
    unsigned int len= get_v(bc);

    if(len && maxlen)
        get_buffer(bc, string, FFMIN(len, maxlen));
    while(len > maxlen){
        get_byte(bc);
        len--;
    }

    if(maxlen)
        string[FFMIN(len, maxlen-1)]= 0;

    if(maxlen == len)
        return -1;
    else
        return 0;
}

static int64_t get_s(ByteIOContext *bc){
    int64_t v = get_v(bc) + 1;

    if (v&1) return -(v>>1);
    else     return  (v>>1);
}

static uint64_t get_fourcc(ByteIOContext *bc){
    unsigned int len= get_v(bc);

    if     (len==2) return get_le16(bc);
    else if(len==4) return get_le32(bc);
    else            return -1;
}

#ifdef TRACE
static inline uint64_t get_v_trace(ByteIOContext *bc, char *file, char *func, int line){
    uint64_t v= get_v(bc);

    printf("get_v %5"PRId64" / %"PRIX64" in %s %s:%d\n", v, v, file, func, line);
    return v;
}

static inline int64_t get_s_trace(ByteIOContext *bc, char *file, char *func, int line){
    int64_t v= get_s(bc);

    printf("get_s %5"PRId64" / %"PRIX64" in %s %s:%d\n", v, v, file, func, line);
    return v;
}

static inline uint64_t get_vb_trace(ByteIOContext *bc, char *file, char *func, int line){
    uint64_t v= get_vb(bc);

    printf("get_vb %5"PRId64" / %"PRIX64" in %s %s:%d\n", v, v, file, func, line);
    return v;
}
#define get_v(bc)  get_v_trace(bc, __FILE__, __PRETTY_FUNCTION__, __LINE__)
#define get_s(bc)  get_s_trace(bc, __FILE__, __PRETTY_FUNCTION__, __LINE__)
#define get_vb(bc)  get_vb_trace(bc, __FILE__, __PRETTY_FUNCTION__, __LINE__)
#endif

static int get_packetheader(NUTContext *nut, ByteIOContext *bc, int calculate_checksum)
{
    int64_t start, size;
//    start= url_ftell(bc) - 8;

    size= get_v(bc);

    init_checksum(bc, calculate_checksum ? av_crc04C11DB7_update : NULL, 1);

//    nut->packet_start[2] = start;
//    nut->written_packet_size= size;

    return size;
}

static int check_checksum(ByteIOContext *bc){
    unsigned long checksum= get_checksum(bc);
//    return checksum != get_be32(bc);

    av_log(NULL, AV_LOG_ERROR, "%08X %08X\n", checksum, (int)get_be32(bc));

    return 0;
}

static uint64_t find_any_startcode(ByteIOContext *bc, int64_t pos){
    uint64_t state=0;

    if(pos >= 0)
        url_fseek(bc, pos, SEEK_SET); //note, this may fail if the stream isnt seekable, but that shouldnt matter, as in this case we simply start where we are currently

    while(!url_feof(bc)){
        state= (state<<8) | get_byte(bc);
        if((state>>56) != 'N')
            continue;
        switch(state){
        case MAIN_STARTCODE:
        case STREAM_STARTCODE:
        case SYNCPOINT_STARTCODE:
        case INFO_STARTCODE:
        case INDEX_STARTCODE:
            return state;
        }
    }

    return 0;
}

/**
 * find the given startcode.
 * @param code the startcode
 * @param pos the start position of the search, or -1 if the current position
 * @returns the position of the startcode or -1 if not found
 */
static int64_t find_startcode(ByteIOContext *bc, uint64_t code, int64_t pos){
    for(;;){
        uint64_t startcode= find_any_startcode(bc, pos);
        if(startcode == code)
            return url_ftell(bc) - 8;
        else if(startcode == 0)
            return -1;
        pos=-1;
    }
}

static int64_t lsb2full(StreamContext *stream, int64_t lsb){
    int64_t mask = (1<<stream->msb_pts_shift)-1;
    int64_t delta= stream->last_pts - mask/2;
    return  ((lsb - delta)&mask) + delta;
}

static int nut_probe(AVProbeData *p){
    int i;
    uint64_t code= 0;

    for (i = 0; i < p->buf_size; i++) {
        code = (code << 8) | p->buf[i];
        if (code == MAIN_STARTCODE)
            return AVPROBE_SCORE_MAX;
    }
    return 0;
}

#define GET_V(dst, check) \
    tmp= get_v(bc);\
    if(!(check)){\
        av_log(s, AV_LOG_ERROR, "Error " #dst " is (%"PRId64")\n", tmp);\
        return -1;\
    }\
    dst= tmp;

static int skip_reserved(ByteIOContext *bc, int64_t pos){
    pos -= url_ftell(bc);

    if(pos<0){
        url_fseek(bc, pos, SEEK_CUR);
        return -1;
    }else{
        while(pos--)
            get_byte(bc);
        return 0;
    }
}

static int decode_main_header(NUTContext *nut){
    AVFormatContext *s= nut->avf;
    ByteIOContext *bc = &s->pb;
    uint64_t tmp, end;
    unsigned int stream_count;
    int i, j, tmp_stream, tmp_mul, tmp_pts, tmp_size, count, tmp_res;

    end= get_packetheader(nut, bc, 1);
    end += url_ftell(bc) - 4;

    GET_V(tmp              , tmp >=2 && tmp <= 3)
    GET_V(stream_count     , tmp > 0 && tmp <=MAX_STREAMS)

    nut->max_distance = get_v(bc);
    if(nut->max_distance > 65536){
        av_log(s, AV_LOG_DEBUG, "max_distance %d\n", nut->max_distance);
        nut->max_distance= 65536;
    }

    GET_V(nut->time_base_count, tmp>0 && tmp<INT_MAX / sizeof(AVRational))
    nut->time_base= av_malloc(nut->time_base_count * sizeof(AVRational));

    for(i=0; i<nut->time_base_count; i++){
        GET_V(nut->time_base[i].num, tmp>0 && tmp<(1ULL<<31))
        GET_V(nut->time_base[i].den, tmp>0 && tmp<(1ULL<<31))
        if(ff_gcd(nut->time_base[i].num, nut->time_base[i].den) != 1){
            av_log(s, AV_LOG_ERROR, "time base invalid\n");
            return -1;
        }
    }
    tmp_pts=0;
    tmp_mul=1;
    tmp_stream=0;
    for(i=0; i<256;){
        int tmp_flags = get_v(bc);
        int tmp_fields= get_v(bc);
        if(tmp_fields>0) tmp_pts   = get_s(bc);
        if(tmp_fields>1) tmp_mul   = get_v(bc);
        if(tmp_fields>2) tmp_stream= get_v(bc);
        if(tmp_fields>3) tmp_size  = get_v(bc);
        else             tmp_size  = 0;
        if(tmp_fields>4) tmp_res   = get_v(bc);
        else             tmp_res   = 0;
        if(tmp_fields>5) count     = get_v(bc);
        else             count     = tmp_mul - tmp_size;

        while(tmp_fields-- > 6)
           get_v(bc);

        if(count == 0 || i+count > 256){
            av_log(s, AV_LOG_ERROR, "illegal count %d at %d\n", count, i);
            return -1;
        }
        if(tmp_stream >= stream_count){
            av_log(s, AV_LOG_ERROR, "illegal stream number\n");
            return -1;
        }

        for(j=0; j<count; j++,i++){
            if (i == 'N') {
                nut->frame_code[i].flags= FLAG_INVALID;
                j--;
                continue;
            }
            nut->frame_code[i].flags           = tmp_flags ;
            nut->frame_code[i].pts_delta       = tmp_pts   ;
            nut->frame_code[i].stream_id       = tmp_stream;
            nut->frame_code[i].size_mul        = tmp_mul   ;
            nut->frame_code[i].size_lsb        = tmp_size+j;
            nut->frame_code[i].reserved_count  = tmp_res   ;
        }
    }
    assert(nut->frame_code['N'].flags == FLAG_INVALID);

    if(skip_reserved(bc, end) || check_checksum(bc)){
        av_log(s, AV_LOG_ERROR, "Main header checksum mismatch\n");
        return -1;
    }

    nut->stream = av_mallocz(sizeof(StreamContext)*stream_count);
    for(i=0; i<stream_count; i++){
        av_new_stream(s, i);
    }

    return 0;
}

static int decode_stream_header(NUTContext *nut){
    AVFormatContext *s= nut->avf;
    ByteIOContext *bc = &s->pb;
    StreamContext *stc;
    int class, nom, denom, stream_id;
    uint64_t tmp, end;
    AVStream *st;

    end= get_packetheader(nut, bc, 1);
    end += url_ftell(bc) - 4;

    GET_V(stream_id, tmp < s->nb_streams && !nut->stream[tmp].time_base.num);
    stc= &nut->stream[stream_id];

    st = s->streams[stream_id];
    if (!st)
        return AVERROR_NOMEM;

    class = get_v(bc);
    tmp = get_fourcc(bc);
    st->codec->codec_tag= tmp;
    switch(class)
    {
        case 0:
            st->codec->codec_type = CODEC_TYPE_VIDEO;
            st->codec->codec_id = codec_get_bmp_id(tmp);
            if (st->codec->codec_id == CODEC_ID_NONE)
                av_log(s, AV_LOG_ERROR, "Unknown codec?!\n");
            break;
        case 1:
            st->codec->codec_type = CODEC_TYPE_AUDIO;
            st->codec->codec_id = codec_get_wav_id(tmp);
            if (st->codec->codec_id == CODEC_ID_NONE)
                av_log(s, AV_LOG_ERROR, "Unknown codec?!\n");
            break;
        case 2:
//            st->codec->codec_type = CODEC_TYPE_TEXT;
//            break;
        case 3:
            st->codec->codec_type = CODEC_TYPE_DATA;
            break;
        default:
            av_log(s, AV_LOG_ERROR, "Unknown stream class (%d)\n", class);
            return -1;
    }
    GET_V(stc->time_base_id    , tmp < nut->time_base_count);
    GET_V(stc->msb_pts_shift   , tmp < 16);
    stc->max_pts_distance= get_v(bc);
    GET_V(stc->decode_delay    , tmp < 1000); //sanity limit, raise this if moors law is true
    st->codec->has_b_frames= stc->decode_delay;
    get_v(bc); //stream flags

    GET_V(st->codec->extradata_size, tmp < (1<<30));
    if(st->codec->extradata_size){
        st->codec->extradata= av_mallocz(st->codec->extradata_size + FF_INPUT_BUFFER_PADDING_SIZE);
        get_buffer(bc, st->codec->extradata, st->codec->extradata_size);
    }

    if (st->codec->codec_type == CODEC_TYPE_VIDEO){
        GET_V(st->codec->width , tmp > 0)
        GET_V(st->codec->height, tmp > 0)
        st->codec->sample_aspect_ratio.num= get_v(bc);
        st->codec->sample_aspect_ratio.den= get_v(bc);
        if((!st->codec->sample_aspect_ratio.num) != (!st->codec->sample_aspect_ratio.den)){
            av_log(s, AV_LOG_ERROR, "invalid aspect ratio\n");
            return -1;
        }
        get_v(bc); /* csp type */
    }else if (st->codec->codec_type == CODEC_TYPE_AUDIO){
        GET_V(st->codec->sample_rate , tmp > 0)
        tmp= get_v(bc); // samplerate_den
        if(tmp > st->codec->sample_rate){
            av_log(s, AV_LOG_ERROR, "bleh, libnut muxed this ;)\n");
            st->codec->sample_rate= tmp;
        }
        GET_V(st->codec->channels, tmp > 0)
    }
    if(skip_reserved(bc, end) || check_checksum(bc)){
        av_log(s, AV_LOG_ERROR, "Stream header %d checksum mismatch\n", stream_id);
        return -1;
    }
    stc->time_base= nut->time_base[stc->time_base_id];
    av_set_pts_info(s->streams[stream_id], 63, stc->time_base.num, stc->time_base.den);
    return 0;
}

static int decode_info_header(NUTContext *nut){
    AVFormatContext *s= nut->avf;
    ByteIOContext *bc = &s->pb;
    uint64_t tmp;
    unsigned int stream_id_plus1, chapter_start, chapter_len, count;
    int chapter_id, i;
    int64_t value, end;
    char name[256], str_value[1024], type_str[256], *type= type_str;

    end= get_packetheader(nut, bc, 1);
    end += url_ftell(bc) - 4;

    GET_V(stream_id_plus1, tmp <= s->nb_streams)
    chapter_id   = get_s(bc);
    chapter_start= get_v(bc);
    chapter_len  = get_v(bc);
    count        = get_v(bc);
    for(i=0; i<count; i++){
        get_str(bc, name, sizeof(name));
        value= get_s(bc);
        if(value == -1){
            type= "UTF-8";
            get_str(bc, str_value, sizeof(str_value));
        }else if(value == -2){
            get_str(bc, type, sizeof(type));
            get_str(bc, str_value, sizeof(str_value));
        }else if(value == -3){
            type= "s";
            value= get_s(bc);
        }else if(value == -4){
            type= "t";
            value= get_v(bc);
        }else if(value < -4){
            type= "r";
            get_s(bc);
        }else{
            type= "v";
        }

        if(chapter_id==0 && !strcmp(type, "UTF-8")){
            if     (!strcmp(name, "Author"))
                pstrcpy(s->author   , sizeof(s->author)   , str_value);
            else if(!strcmp(name, "Title"))
                pstrcpy(s->title    , sizeof(s->title)    , str_value);
            else if(!strcmp(name, "Copyright"))
                pstrcpy(s->copyright, sizeof(s->copyright), str_value);
            else if(!strcmp(name, "Description"))
                pstrcpy(s->comment  , sizeof(s->comment)  , str_value);
        }
    }

    if(skip_reserved(bc, end) || check_checksum(bc)){
        av_log(s, AV_LOG_ERROR, "Info header checksum mismatch\n");
        return -1;
    }
    return 0;
}

static int decode_syncpoint(NUTContext *nut){
    AVFormatContext *s= nut->avf;
    ByteIOContext *bc = &s->pb;
    int64_t end;
    uint64_t tmp;
    int i;
    AVRational time_base;

    nut->last_syncpoint_pos= url_ftell(bc);

    end= get_packetheader(nut, bc, 1);
    end += url_ftell(bc) - 4;

    tmp= get_v(bc);
    get_v(bc); //back_ptr_div16

    time_base= nut->time_base[tmp % nut->time_base_count];
    for(i=0; i<s->nb_streams; i++){
        nut->stream[i].last_pts= av_rescale_q(tmp / nut->time_base_count, time_base, nut->stream[i].time_base); //FIXME rounding and co
        //last_key_frame ?
    }
    //FIXME put this in a reset func maybe

    if(skip_reserved(bc, end) || check_checksum(bc)){
        av_log(s, AV_LOG_ERROR, "Info header checksum mismatch\n");
        return -1;
    }
    return 0;
}

static int nut_read_header(AVFormatContext *s, AVFormatParameters *ap)
{
    NUTContext *nut = s->priv_data;
    ByteIOContext *bc = &s->pb;
    int64_t pos;
    int inited_stream_count;

    nut->avf= s;

    /* main header */
    pos=0;
    for(;;){
        pos= find_startcode(bc, MAIN_STARTCODE, pos)+1;
        if (pos<0+1){
            av_log(s, AV_LOG_ERROR, "no main startcode found\n");
            return -1;
        }
        if(decode_main_header(nut) >= 0)
            break;
    }

    /* stream headers */
    pos=0;
    for(inited_stream_count=0; inited_stream_count < s->nb_streams;){
        pos= find_startcode(bc, STREAM_STARTCODE, pos)+1;
        if (pos<0+1){
            av_log(s, AV_LOG_ERROR, "not all stream headers found\n");
            return -1;
        }
        if(decode_stream_header(nut) >= 0)
            inited_stream_count++;
    }

    /* info headers */
    pos=0;
    for(;;){
        uint64_t startcode= find_any_startcode(bc, pos);
        pos= url_ftell(bc);

        if(startcode==0){
            av_log(s, AV_LOG_ERROR, "EOF before video frames\n");
            return -1;
        }else if(startcode == SYNCPOINT_STARTCODE){
            nut->next_startcode= startcode;
            break;
        }else if(startcode != INFO_STARTCODE){
            continue;
        }

        decode_info_header(nut);
    }

    return 0;
}

static int decode_frame_header(NUTContext *nut, int *flags_ret, int64_t *pts, int *stream_id, int frame_code){
    AVFormatContext *s= nut->avf;
    ByteIOContext *bc = &s->pb;
    StreamContext *stc;
    int size, flags, size_mul, pts_delta, i, reserved_count;
    uint64_t tmp;

    if(url_ftell(bc) > nut->last_syncpoint_pos + nut->max_distance){
        av_log(s, AV_LOG_ERROR, "last frame must have been damaged\n");
        return -1;
    }

    flags          = nut->frame_code[frame_code].flags;
    size_mul       = nut->frame_code[frame_code].size_mul;
    size           = nut->frame_code[frame_code].size_lsb;
    *stream_id     = nut->frame_code[frame_code].stream_id;
    pts_delta      = nut->frame_code[frame_code].pts_delta;
    reserved_count = nut->frame_code[frame_code].reserved_count;

    if(flags & FLAG_INVALID)
        return -1;
    if(flags & FLAG_CODED)
        flags ^= get_v(bc);
    if(flags & FLAG_STREAM_ID){
        GET_V(*stream_id, tmp < s->nb_streams)
    }
    stc= &nut->stream[*stream_id];
    if(flags&FLAG_CODED_PTS){
        int coded_pts= get_v(bc);
//FIXME check last_pts validity?
        if(coded_pts < (1<<stc->msb_pts_shift)){
            *pts=lsb2full(stc, coded_pts);
        }else
            *pts=coded_pts - (1<<stc->msb_pts_shift);
    }else
        *pts= stc->last_pts + pts_delta;
    if(flags&FLAG_SIZE_MSB){
        size += size_mul*get_v(bc);
    }
    if(flags&FLAG_RESERVED)
        reserved_count= get_v(bc);
    for(i=0; i<reserved_count; i++)
        get_v(bc);
    if(flags&FLAG_CHECKSUM){
        get_be32(bc); //FIXME check this
    }
    *flags_ret= flags;

    stc->last_pts= *pts;
    stc->last_key_frame= flags&FLAG_KEY; //FIXME change to last flags

    return size;
}

static int decode_frame(NUTContext *nut, AVPacket *pkt, int frame_code){
    AVFormatContext *s= nut->avf;
    ByteIOContext *bc = &s->pb;
    int size, stream_id, flags, discard;
    int64_t pts, last_IP_pts;

    size= decode_frame_header(nut, &flags, &pts, &stream_id, frame_code);
    if(size < 0)
        return -1;

    discard= s->streams[ stream_id ]->discard;
    last_IP_pts= s->streams[ stream_id ]->last_IP_pts;
    if(  (discard >= AVDISCARD_NONKEY && !(flags & FLAG_KEY))
       ||(discard >= AVDISCARD_BIDIR && last_IP_pts != AV_NOPTS_VALUE && last_IP_pts > pts)
       || discard >= AVDISCARD_ALL){
        url_fskip(bc, size);
        return 1;
    }

    av_get_packet(bc, pkt, size);
    pkt->stream_index = stream_id;
    if (flags & FLAG_KEY)
        pkt->flags |= PKT_FLAG_KEY;
    pkt->pts = pts;

    return 0;
}

static int nut_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NUTContext *nut = s->priv_data;
    ByteIOContext *bc = &s->pb;
    int i, frame_code=0, ret, skip;

    for(;;){
        int64_t pos= url_ftell(bc);
        uint64_t tmp= nut->next_startcode;
        nut->next_startcode=0;

        if (url_feof(bc))
            return -1;

        if(tmp){
            pos-=8;
        }else{
            frame_code = get_byte(bc);
            if(frame_code == 'N'){
                tmp= frame_code;
                for(i=1; i<8; i++)
                    tmp = (tmp<<8) + get_byte(bc);
            }
        }
        switch(tmp){
        case MAIN_STARTCODE:
        case STREAM_STARTCODE:
        case INDEX_STARTCODE:
            skip= get_packetheader(nut, bc, 0);
            url_fseek(bc, skip, SEEK_CUR);
            break;
        case INFO_STARTCODE:
            if(decode_info_header(nut)<0)
                goto resync;
            break;
        case SYNCPOINT_STARTCODE:
            if(decode_syncpoint(nut)<0)
                goto resync;
            frame_code = get_byte(bc);
        case 0:
            ret= decode_frame(nut, pkt, frame_code);
            if(ret==0)
                return 0;
            else if(ret==1) //ok but discard packet
                break;
        default:
resync:
av_log(s, AV_LOG_DEBUG, "syncing from %"PRId64"\n", pos);
            tmp= find_any_startcode(bc, pos+1);
            if(tmp==0)
                return -1;
av_log(s, AV_LOG_DEBUG, "sync\n");
            nut->next_startcode= tmp;
        }
    }
}

static int nut_read_close(AVFormatContext *s)
{
    NUTContext *nut = s->priv_data;

    av_freep(&nut->time_base);
    av_freep(&nut->stream);

    return 0;
}

#ifdef CONFIG_NUT_DEMUXER
AVInputFormat nut_demuxer = {
    "nut",
    "nut format",
    sizeof(NUTContext),
    nut_probe,
    nut_read_header,
    nut_read_packet,
    nut_read_close,
    NULL,
    NULL,
    .extensions = "nut",
};
#endif
