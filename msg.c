/**
 * @file msg.c
 * @note Copyright (C) 2011 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <threads.h>

#include "contain.h"
#include "msg.h"
#include "print.h"
#include "tlv.h"

_Thread_local int assume_two_step = 0;

/*
 * Head room fits a VLAN Ethernet header, and 'msg' is 64 bit aligned.
 */
#define MSG_HEADROOM 24

struct message_storage {
	unsigned char reserved[MSG_HEADROOM];
	struct ptp_message msg __attribute__((aligned (8)));
};

static _Thread_local TAILQ_HEAD(msg_pool, ptp_message) msg_pool; // put init to thread thing

/* TODO: need to initialize once */
void t_msg_pool_init() {
    msg_pool.tqh_first = (void *)0;
    msg_pool.tqh_last = &(msg_pool).tqh_first;
}

/* = TAILQ_HEAD_INITIALIZER(msg_pool);
static struct msg_pool
        { struct ptp_message *tqh_first;
          struct ptp_message **tqh_last; }
            msg_pool =
                    { ((void *)0), &(msg_pool).tqh_first };*/
struct pool_stat_type {
	int total;
	int count;
};

static _Thread_local struct pool_stat_type pool_stats;

#ifdef DEBUG_POOL
static void pool_debug(const char *str, void *addr)
{
#ifdef DEBUG
	pr_debug("*** %p %10s total %d count %d used %d\n",
#else
	fprintf(stderr, "*** %p %10s total %d count %d used %d\n",
#endif
		addr, str, pool_stats.total, pool_stats.count,
		pool_stats.total - pool_stats.count);
}
#else
static void pool_debug(const char *str, void *addr)
{
}
#endif

static void announce_pre_send(struct announce_msg *m)
{
	m->currentUtcOffset = htons(m->currentUtcOffset);
	m->grandmasterClockQuality.offsetScaledLogVariance =
		htons(m->grandmasterClockQuality.offsetScaledLogVariance);
	m->stepsRemoved = htons(m->stepsRemoved);
}

static void announce_post_recv(struct announce_msg *m)
{
	m->currentUtcOffset = ntohs(m->currentUtcOffset);
	m->grandmasterClockQuality.offsetScaledLogVariance =
		ntohs(m->grandmasterClockQuality.offsetScaledLogVariance);
	m->stepsRemoved = ntohs(m->stepsRemoved);
}

static int hdr_post_recv(struct ptp_header *m)
{
	if ((m->ver & MAJOR_VERSION_MASK) != PTP_MAJOR_VERSION)
		return -EPROTO;
	m->messageLength = ntohs(m->messageLength);
	m->correction = net2host64(m->correction);
	m->sourcePortIdentity.portNumber = ntohs(m->sourcePortIdentity.portNumber);
	m->sequenceId = ntohs(m->sequenceId);
#if 0
#ifdef VIRT_EVENT
#ifdef DEBUG
    if (msg_domainNumber(m) == 1 && (msg_type(m) == SYNC || msg_type(m) == FOLLOW_UP)) {
        pr_debug("[VIRT_EVENT] DEBUG HDR: %p", m);
        msg_print(m, stderr);
    }
#endif
#endif
#endif
	return 0;
}

static int hdr_pre_send(struct ptp_header *m)
{
	m->messageLength = htons(m->messageLength);
	m->correction = host2net64(m->correction);
	m->sourcePortIdentity.portNumber = htons(m->sourcePortIdentity.portNumber);
	m->sequenceId = htons(m->sequenceId);
	return 0;
}

static uint8_t *msg_suffix(struct ptp_message *m)
{
	switch (msg_type(m)) {
	case SYNC:
		return NULL;
	case DELAY_REQ:
		return m->delay_req.suffix;
	case PDELAY_REQ:
		return NULL;
	case PDELAY_RESP:
		return NULL;
	case FOLLOW_UP:
		return m->follow_up.suffix;
	case DELAY_RESP:
		return m->delay_resp.suffix;
	case PDELAY_RESP_FOLLOW_UP:
		return m->pdelay_resp_fup.suffix;
	case ANNOUNCE:
		return m->announce.suffix;
	case SIGNALING:
		return m->signaling.suffix;
	case MANAGEMENT:
		return m->management.suffix;
	}
	return NULL;
}

static struct tlv_extra *msg_tlv_prepare(struct ptp_message *msg, int length)
{
	struct tlv_extra *extra, *tmp;
	uint8_t *ptr;

	/* Make sure this message type admits appended TLVs. */
	ptr = msg_suffix(msg);
	if (!ptr) {
		pr_err("TLV on %s not allowed", msg_type_string(msg_type(msg)));
		return NULL;
	}
	tmp = TAILQ_LAST(&msg->tlv_list, tlv_list);
	if (tmp) {
		ptr = (uint8_t *) tmp->tlv;
		ptr += sizeof(tmp->tlv->type);
		ptr += sizeof(tmp->tlv->length);
		ptr += tmp->tlv->length;
	}

	/* Check that the message buffer has enough room for the new TLV. */
	if ((unsigned long)(ptr + length) >
	    (unsigned long)(&msg->tail_room)) {
		pr_debug("cannot fit TLV of length %d into message", length);
		return NULL;
	}

	/* Allocate a TLV descriptor and setup the pointer. */
	extra = tlv_extra_alloc();
	if (!extra) {
		pr_err("failed to allocate TLV descriptor");
		return NULL;
	}
	extra->tlv = (struct TLV *) ptr;

	return extra;
}

static void msg_tlv_recycle(struct ptp_message *msg)
{
	struct tlv_extra *extra;

	while ((extra = TAILQ_FIRST(&msg->tlv_list)) != NULL) {
		TAILQ_REMOVE(&msg->tlv_list, extra, list);
		tlv_extra_recycle(extra);
	}
}

static void port_id_post_recv(struct PortIdentity *pid)
{
	pid->portNumber = ntohs(pid->portNumber);
}

static void port_id_pre_send(struct PortIdentity *pid)
{
	pid->portNumber = htons(pid->portNumber);
}

static int suffix_post_recv(struct ptp_message *msg, int len)
{
	uint8_t *ptr = msg_suffix(msg);
	struct tlv_extra *extra;
	int err, suffix_len = 0;

	if (!ptr)
		return 0;

	while (len >= sizeof(struct TLV)) {
		extra = tlv_extra_alloc();
		if (!extra) {
			pr_err("failed to allocate TLV descriptor");
			return -ENOMEM;
		}
		extra->tlv = (struct TLV *) ptr;
		extra->tlv->type = ntohs(extra->tlv->type);
		extra->tlv->length = ntohs(extra->tlv->length);
		if (extra->tlv->length % 2) {
			tlv_extra_recycle(extra);
			return -EBADMSG;
		}
		suffix_len += sizeof(struct TLV);
		len -= sizeof(struct TLV);
		ptr += sizeof(struct TLV);
		if (extra->tlv->length > len) {
			tlv_extra_recycle(extra);
			return -EBADMSG;
		}
		suffix_len += extra->tlv->length;
		len -= extra->tlv->length;
		ptr += extra->tlv->length;
		err = tlv_post_recv(extra);
		if (err) {
			tlv_extra_recycle(extra);
			return err;
		}
		msg_tlv_attach(msg, extra);
	}
	return suffix_len;
}

static void suffix_pre_send(struct ptp_message *msg)
{
	struct tlv_extra *extra;
	struct TLV *tlv;

	TAILQ_FOREACH(extra, &msg->tlv_list, list) {
		tlv = extra->tlv;
		tlv_pre_send(tlv, extra);
		tlv->type = htons(tlv->type);
		tlv->length = htons(tlv->length);
	}
	msg_tlv_recycle(msg);
}

static void timestamp_post_recv(struct ptp_message *m, struct Timestamp *ts)
{
	uint32_t lsb = ntohl(ts->seconds_lsb);
	uint16_t msb = ntohs(ts->seconds_msb);

	m->ts.pdu.sec  = ((uint64_t)lsb) | (((uint64_t)msb) << 32);
	m->ts.pdu.nsec = ntohl(ts->nanoseconds);
}

static void timestamp_pre_send(struct Timestamp *ts)
{
	ts->seconds_lsb = htonl(ts->seconds_lsb);
	ts->seconds_msb = htons(ts->seconds_msb);
	ts->nanoseconds = htonl(ts->nanoseconds);
}

/* public methods */

struct ptp_message *msg_allocate(void)
{
	struct message_storage *s;
	struct ptp_message *m = TAILQ_FIRST(&msg_pool);

	if (m) {
		TAILQ_REMOVE(&msg_pool, m, list);
		pool_stats.count--;
		pool_debug("dequeue", m);
	} else {
		s = malloc(sizeof(*s));
		if (s) {
			m = &s->msg;
			pool_stats.total++;
			pool_debug("allocate", m);
		}
	}
	if (m) {
		memset(m, 0, sizeof(*m));
		m->refcnt = 1;
		TAILQ_INIT(&m->tlv_list);
	}

	return m;
}

void msg_cleanup(void)
{
	struct message_storage *s;
	struct ptp_message *m;

	tlv_extra_cleanup();

	while ((m = TAILQ_FIRST(&msg_pool)) != NULL) {
		TAILQ_REMOVE(&msg_pool, m, list);
		s = container_of(m, struct message_storage, msg);
		free(s);
	}
}

struct ptp_message *msg_duplicate_off_pool(struct ptp_message *msg, struct ptp_message *target, int cnt, int *target_cnt)
{
	msg_tlv_recycle(target);
	memcpy(target, msg, sizeof(*target)); // source Q list info can be ignored
    // TODO: copy tlv_list, append one by one...

    *target_cnt = cnt;

    return target;
}

struct ptp_message *msg_cp_data_and_ts(struct ptp_message *src, struct ptp_message *target, int cnt)
{
    // backup other data
	TAILQ_ENTRY(ptp_message) list;

    memcpy(&list, &(src->list), sizeof(list)); // backup the source Q list info
	int tail_room = target->tail_room;
	int refcnt =  target->refcnt;


	memcpy(target, src, sizeof(*target)); // for tlv, it also need to handle

	target->tail_room = tail_room;
	target->refcnt = refcnt;
	memcpy(&(target->list), &list, sizeof(list)); // restore Q list info

    return target;
}

struct ptp_message *msg_duplicate(struct ptp_message *msg, int cnt, UInteger8 domainNumber)
{
	struct ptp_message *dup;
	int err;

	dup = msg_allocate();
	if (!dup) {
		return NULL;
	}
	memcpy(dup, msg, sizeof(*dup));
	dup->refcnt = 1;
	TAILQ_INIT(&dup->tlv_list);

	err = msg_post_recv(dup, cnt, domainNumber);
	if (err) {
		switch (err) {
		case -EBADMSG:
			pr_err("msg_duplicate: bad message");
			break;
		case -EPROTO:
			pr_debug("msg_duplicate: ignoring message");
			break;
		}
		msg_put(dup);
		return NULL;
	}
	if (msg_sots_missing(msg)) {
		pr_err("msg_duplicate: received %s without timestamp",
		       msg_type_string(msg_type(msg)));
		msg_put(dup);
		return NULL;
	}

	return dup;
}

void msg_get(struct ptp_message *m)
{
	m->refcnt++;
}

int msg_post_recv(struct ptp_message *m, int cnt, UInteger8 domainNumber)
{
	int err, pdulen, suffix_len, type;

    //ref: https://elixir.bootlin.com/linux/latest/source/include/uapi/asm-generic/errno.h#L57
	if (cnt < sizeof(struct ptp_header))
		return -EBADMSG;

	type = msg_type(m);

#ifdef VIRT_EVENT
	if ((domainNumber != -1) && (msg_domainNumber(m) != domainNumber)) {
        //pr_debug("[VIRT_EVENT] different domain");
        switch (type) {
        case SYNC:
            return -ENOMSG; // will copy
        case DELAY_REQ:
            break;
        case PDELAY_REQ:
            break;
        case PDELAY_RESP:
            return -ENOMSG; // will copy
        case FOLLOW_UP:
            return -ENOMSG; // will copy
        case DELAY_RESP:
            return -ENOMSG; // will copy
        case PDELAY_RESP_FOLLOW_UP:
            return -ENOMSG; // will copy
        case ANNOUNCE:
            break;
        case SIGNALING:
            break;
        case MANAGEMENT:
            break;
        default:
            return -EBADMSG;
        }
        return -EPROTO; // ignore
    }
#endif
	err = hdr_post_recv(&m->header); // this will ntohs
	if (err)
		return err;
	switch (type) {
	case SYNC:
		pdulen = sizeof(struct sync_msg);
		break;
	case DELAY_REQ:
		pdulen = sizeof(struct delay_req_msg);
		break;
	case PDELAY_REQ:
		pdulen = sizeof(struct pdelay_req_msg);
		break;
	case PDELAY_RESP:
		pdulen = sizeof(struct pdelay_resp_msg);
		break;
	case FOLLOW_UP:
		pdulen = sizeof(struct follow_up_msg);
		break;
	case DELAY_RESP:
		pdulen = sizeof(struct delay_resp_msg);
		break;
	case PDELAY_RESP_FOLLOW_UP:
		pdulen = sizeof(struct pdelay_resp_fup_msg);
		break;
	case ANNOUNCE:
		pdulen = sizeof(struct announce_msg);
		break;
	case SIGNALING:
		pdulen = sizeof(struct signaling_msg);
		break;
	case MANAGEMENT:
		pdulen = sizeof(struct management_msg);
		break;
	default:
		return -EBADMSG;
	}

	if (cnt < pdulen)
		return -EBADMSG;

	switch (type) {
	case SYNC:
		timestamp_post_recv(m, &m->sync.originTimestamp);
		break;
	case DELAY_REQ:
		break;
	case PDELAY_REQ:
		break;
	case PDELAY_RESP:
		timestamp_post_recv(m, &m->pdelay_resp.requestReceiptTimestamp);
		port_id_post_recv(&m->pdelay_resp.requestingPortIdentity);
		break;
	case FOLLOW_UP:
		timestamp_post_recv(m, &m->follow_up.preciseOriginTimestamp);
		break;
	case DELAY_RESP:
		timestamp_post_recv(m, &m->delay_resp.receiveTimestamp);
		port_id_post_recv(&m->delay_resp.requestingPortIdentity);
		break;
	case PDELAY_RESP_FOLLOW_UP:
		timestamp_post_recv(m, &m->pdelay_resp_fup.responseOriginTimestamp);
		port_id_post_recv(&m->pdelay_resp_fup.requestingPortIdentity);
		break;
	case ANNOUNCE:
		clock_gettime(CLOCK_MONOTONIC, &m->ts.host);
		timestamp_post_recv(m, &m->announce.originTimestamp);
		announce_post_recv(&m->announce);
		break;
	case SIGNALING:
		port_id_post_recv(&m->signaling.targetPortIdentity);
		break;
	case MANAGEMENT:
		port_id_post_recv(&m->management.targetPortIdentity);
		break;
	}

	suffix_len = suffix_post_recv(m, cnt - pdulen);
	if (suffix_len < 0) {
		return suffix_len;
	}
	if (pdulen + suffix_len != m->header.messageLength) {
		return -EBADMSG;
	}

	return 0;
}

int msg_pre_send(struct ptp_message *m)
{
	int type;

	if (hdr_pre_send(&m->header))
		return -1;

	type = msg_type(m);

	switch (type) {
	case SYNC:
		break;
	case DELAY_REQ:
		clock_gettime(CLOCK_MONOTONIC, &m->ts.host);
		break;
	case PDELAY_REQ:
		break;
	case PDELAY_RESP:
		timestamp_pre_send(&m->pdelay_resp.requestReceiptTimestamp);
		port_id_pre_send(&m->pdelay_resp.requestingPortIdentity);
		break;
	case FOLLOW_UP:
		timestamp_pre_send(&m->follow_up.preciseOriginTimestamp);
		break;
	case DELAY_RESP:
		timestamp_pre_send(&m->delay_resp.receiveTimestamp);
		m->delay_resp.requestingPortIdentity.portNumber =
			htons(m->delay_resp.requestingPortIdentity.portNumber);
		break;
	case PDELAY_RESP_FOLLOW_UP:
		timestamp_pre_send(&m->pdelay_resp_fup.responseOriginTimestamp);
		port_id_pre_send(&m->pdelay_resp_fup.requestingPortIdentity);
		break;
	case ANNOUNCE:
		announce_pre_send(&m->announce);
		break;
	case SIGNALING:
		port_id_pre_send(&m->signaling.targetPortIdentity);
		break;
	case MANAGEMENT:
		port_id_pre_send(&m->management.targetPortIdentity);
		break;
	default:
		return -1;
	}
	suffix_pre_send(m);
	return 0;
}

struct tlv_extra *msg_tlv_append(struct ptp_message *msg, int length)
{
	struct tlv_extra *extra;

	extra = msg_tlv_prepare(msg, length);
	if (extra) {
		msg->header.messageLength += length;
		msg_tlv_attach(msg, extra);
	}
	return extra;
}

void msg_tlv_attach(struct ptp_message *msg, struct tlv_extra *extra)
{
	TAILQ_INSERT_TAIL(&msg->tlv_list, extra, list);
}

int msg_tlv_count(struct ptp_message *msg)
{
	int count = 0;
	struct tlv_extra *extra;

	for (extra = TAILQ_FIRST(&msg->tlv_list);
			extra != NULL;
			extra = TAILQ_NEXT(extra, list))
		count++;

	return count;
}

const char *msg_type_string(int type)
{
	switch (type) {
	case SYNC:
		return "SYNC";
	case DELAY_REQ:
		return "DELAY_REQ";
	case PDELAY_REQ:
		return "PDELAY_REQ";
	case PDELAY_RESP:
		return "PDELAY_RESP";
	case FOLLOW_UP:
		return "FOLLOW_UP";
	case DELAY_RESP:
		return "DELAY_RESP";
	case PDELAY_RESP_FOLLOW_UP:
		return "PDELAY_RESP_FOLLOW_UP";
	case ANNOUNCE:
		return "ANNOUNCE";
	case SIGNALING:
		return "SIGNALING";
	case MANAGEMENT:
		return "MANAGEMENT";
	}
	return "unknown";
}

void msg_print(struct ptp_message *m, FILE *fp)
{
	fprintf(fp,
		"\t"
		"pointerLoc %p "
		"%-10s "
//		"versionPTP         0x%02X "
		"messageLength      %hu "
		"domainNumber       %u "
//		"reserved1          0x%02X "
//		"flagField          0x%02X%02X "
//		"correction         %lld "
//		"reserved2          %u "
//		"sourcePortIdentity ... "
		"sequenceId %4hu "
//		"control            %u "
//		"logMessageInterval %d "
		,
		m,
		msg_type_string(msg_type(m)),
//		m->header.ver,
		m->header.messageLength,
		m->header.domainNumber,
//		m->header.reserved1,
//		m->header.flagField[0],
//		m->header.flagField[1],
//		m->header.correction,
//		m->header.reserved2,
//		m->header.sourcePortIdentity,
		m->header.sequenceId
//		m->header.control,
//		m->header.logMessageInterval
		);
	fprintf(fp, "\n");
}

void msg_put(struct ptp_message *m)
{
	m->refcnt--;
	if (m->refcnt) {
		return;
	}
	pool_stats.count++;
	pool_debug("recycle", m);
	msg_tlv_recycle(m);
	TAILQ_INSERT_HEAD(&msg_pool, m, list);
}

int msg_sots_missing(struct ptp_message *m)
{
	int type = msg_type(m);
	switch (type) {
	case SYNC:
	case DELAY_REQ:
	case PDELAY_REQ:
	case PDELAY_RESP:
		break;
	case FOLLOW_UP:
	case DELAY_RESP:
	case PDELAY_RESP_FOLLOW_UP:
	case ANNOUNCE:
	case SIGNALING:
	case MANAGEMENT:
	default:
		return 0;
	}
	return msg_sots_valid(m) ? 0 : 1;
}
