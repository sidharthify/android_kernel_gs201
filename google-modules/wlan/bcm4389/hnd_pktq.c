/*
 * HND generic pktq operation primitives
 *
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

#include <typedefs.h>
#include <osl.h>
#include <osl_ext.h>
#include <bcmutils.h>
#include <hnd_pktq.h>

/* mutex macros for thread safe */
#ifdef HND_PKTQ_THREAD_SAFE
#define HND_PKTQ_MUTEX_CREATE(name, mutex)	osl_ext_mutex_create(name, mutex)
#define HND_PKTQ_MUTEX_DELETE(mutex)		osl_ext_mutex_delete(mutex)
#define HND_PKTQ_MUTEX_ACQUIRE(mutex, msec)	osl_ext_mutex_acquire(mutex, msec)
#define HND_PKTQ_MUTEX_RELEASE(mutex)		osl_ext_mutex_release(mutex)
#else
#define HND_PKTQ_MUTEX_CREATE(name, mutex)	OSL_EXT_SUCCESS
#define HND_PKTQ_MUTEX_DELETE(mutex)		OSL_EXT_SUCCESS
#define HND_PKTQ_MUTEX_ACQUIRE(mutex, msec)	OSL_EXT_SUCCESS
#define HND_PKTQ_MUTEX_RELEASE(mutex)		OSL_EXT_SUCCESS
#endif /* HND_PKTQ_THREAD_SAFE */

/* status during txfifo sync */
#if defined(PROP_TXSTATUS)
#define TXQ_PKT_DEL		0x01
#define HEAD_PKT_FLUSHED	0xFF
#endif /* defined(PROP_TXSTATUS) */
/*
 * osl multiple-precedence packet queue
 * hi_prec is always >= the number of the highest non-empty precedence
 */
void *
BCMPOSTTRAPFASTPATH(pktq_penq)(struct pktq *pq, int prec, void *p)
{
	struct pktq_prec *q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);
	ASSERT_FP(PKTLINK(p) == NULL);		/* queueing chains not allowed */

	ASSERT_FP(!pktq_full(pq));
	ASSERT_FP(!pktqprec_full(pq, prec));

	PKTSETQCALLER(p, pq, CALL_SITE);

	q = &pq->q[prec];

	if (q->head)
		PKTSETLINK(q->tail, p);
	else
		q->head = p;

	q->tail = p;
	q->n_pkts++;

	pq->n_pkts_tot++;

	if (pq->hi_prec < prec)
		pq->hi_prec = (uint8)prec;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

void *
BCMFASTPATH(spktq_enq_chain)(struct spktq *dspq, struct spktq *sspq)
{
	struct pktq_prec *dq;
	struct pktq_prec *sq;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&dspq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&sspq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	dq = &dspq->q;
	sq = &sspq->q;

	if (dq->head) {
		PKTSETLINK(OSL_PHYS_TO_VIRT_ADDR(dq->tail), OSL_VIRT_TO_PHYS_ADDR(sq->head));
	}
	else {
		dq->head = sq->head;
	}

	dq->tail = sq->tail;
	dq->n_pkts += sq->n_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&dspq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&sspq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return dspq;
}

/*
 * osl simple, non-priority packet queue
 */
void *
BCMFASTPATH(spktq_enq)(struct spktq *spq, void *p)
{
	struct pktq_prec *q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(!spktq_full(spq));

	PKTSETQCALLER(p, spq, CALL_SITE);

	PKTSETLINK(p, NULL);

	q = &spq->q;

	if (q->head)
		PKTSETLINK(q->tail, p);
	else
		q->head = p;

	q->tail = p;
	q->n_pkts++;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

void *
BCMPOSTTRAPFASTPATH(pktq_penq_head)(struct pktq *pq, int prec, void *p)
{
	struct pktq_prec *q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);
	ASSERT_FP(PKTLINK(p) == NULL);		/* queueing chains not allowed */

	ASSERT_FP(!pktq_full(pq));
	ASSERT_FP(!pktqprec_full(pq, prec));

	PKTSETQCALLER(p, pq, CALL_SITE);

	q = &pq->q[prec];

	if (q->head == NULL)
		q->tail = p;

	PKTSETLINK(p, q->head);
	q->head = p;
	q->n_pkts++;

	pq->n_pkts_tot++;

	if (pq->hi_prec < prec)
		pq->hi_prec = (uint8)prec;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

void *
BCMFASTPATH(spktq_enq_head)(struct spktq *spq, void *p)
{
	struct pktq_prec *q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(!spktq_full(spq));

	PKTSETQCALLER(p, spq, CALL_SITE);
	PKTSETLINK(p, NULL);

	q = &spq->q;

	if (q->head == NULL)
		q->tail = p;

	PKTSETLINK(p, q->head);
	q->head = p;
	q->n_pkts++;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

void *
BCMFASTPATH(pktq_pdeq)(struct pktq *pq, int prec)
{
	struct pktq_prec *q;
	void *p;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);

	q = &pq->q[prec];

	if ((p = q->head) == NULL)
		goto done;

	if ((q->head = PKTLINK(p)) == NULL)
		q->tail = NULL;

	q->n_pkts--;

	pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif

	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, pq, CALL_SITE);
	return p;
}

void *
BCMFASTPATH(spktq_deq)(struct spktq *spq)
{
	struct pktq_prec *q;
	void *p;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	q = &spq->q;

	if ((p = q->head) == NULL)
		goto done;

	if ((q->head = PKTLINK(p)) == NULL)
		q->tail = NULL;

	q->n_pkts--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif

	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, spq, CALL_SITE);
	return p;
}

void *
BCMFASTPATH(spktq_delete_node)(struct spktq *spq, void *prev, void *cur)
{
	struct pktq_prec *q;
	void *next = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS) {
		return NULL;
	}

	q = &spq->q;

	if (cur == q->head) {
		spktq_deq(spq);
		next = q->head;
		goto done;
	}

	if ((cur == NULL) || (prev == NULL)) {
		ASSERT(0);
		next = NULL;
		goto done;
	}

	ASSERT_FP(PKTLINK(prev) == cur);
	next = PKTLINK(cur);
	PKTSETLINK(prev, next);
	PKTSETLINK(cur, NULL);
	ASSERT_FP(q->n_pkts);
	q->n_pkts--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif
done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS) {
		return NULL;
	}
	return next;
}

void*
BCMFASTPATH(spktq_deq_virt)(struct spktq *spq)
{
	struct pktq_prec *q;
	void *p;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	q = &spq->q;

	if ((p = q->head) == NULL)
		goto done;

	p = (void *)OSL_PHYS_TO_VIRT_ADDR(p);

	if ((q->head = (void*)PKTLINK(p)) == NULL)
		q->tail = NULL;

	q->n_pkts--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif

	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, spq, CALL_SITE);
	return p;
}

void *
BCMFASTPATH(pktq_pdeq_tail)(struct pktq *pq, int prec)
{
	struct pktq_prec *q;
	void *p, *prev;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);

	q = &pq->q[prec];

	if ((p = q->head) == NULL)
		goto done;

	for (prev = NULL; p != q->tail; p = PKTLINK(p))
		prev = p;

	if (prev)
		PKTSETLINK(prev, NULL);
	else
		q->head = NULL;

	q->tail = prev;
	q->n_pkts--;

	pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif
done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, pq, CALL_SITE);
	return p;
}

void *
BCMFASTPATH(spktq_deq_tail)(struct spktq *spq)
{
	struct pktq_prec *q;
	void *p, *prev;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	q = &spq->q;

	if ((p = q->head) == NULL)
		goto done;

	for (prev = NULL; p != q->tail; p = PKTLINK(p))
		prev = p;

	if (prev)
		PKTSETLINK(prev, NULL);
	else
		q->head = NULL;

	q->tail = prev;
	q->n_pkts--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif
done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, spq, CALL_SITE);
	return p;
}

void *
pktq_peek_tail(struct pktq *pq, int *prec_out)
{
	int prec;
	void *p = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	if (pq->n_pkts_tot == 0)
		goto done;

	for (prec = 0; prec < pq->hi_prec; prec++)
		if (pq->q[prec].head)
			break;

	if (prec_out)
		*prec_out = prec;

	p = pq->q[prec].tail;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

/*
 * Append spktq 'list' to the tail of pktq 'pq'
 */
void
BCMFASTPATH(pktq_append)(struct pktq *pq, int prec, struct spktq *list)
{
	struct pktq_prec *q;
	struct pktq_prec *list_q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	list_q = &list->q;

	PKTSETQCALLER_LIST(list_q->head, list_q->n_pkts, pq, CALL_SITE);

	/* empty list check */
	if (list_q->head == NULL)
		goto done;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);
	ASSERT_FP(PKTLINK(list_q->tail) == NULL);         /* terminated list */

	ASSERT_FP(!pktq_full(pq));
	ASSERT_FP(!pktqprec_full(pq, prec));

	q = &pq->q[prec];

	if (q->head)
		PKTSETLINK(q->tail, list_q->head);
	else
		q->head = list_q->head;

	q->tail = list_q->tail;
	q->n_pkts += list_q->n_pkts;
	pq->n_pkts_tot += list_q->n_pkts;

	if (pq->hi_prec < prec)
		pq->hi_prec = (uint8)prec;

#ifdef WL_TXQ_STALL
	list_q->dequeue_count += list_q->n_pkts;
#endif

	list_q->head = NULL;
	list_q->tail = NULL;
	list_q->n_pkts = 0;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return;
}

/*
 * Append spktq 'list' to the tail of spktq 'spq'
 */
void
BCMFASTPATH(spktq_append)(struct spktq *spq, struct spktq *list)
{
	struct pktq_prec *q;
	struct pktq_prec *list_q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	list_q = &list->q;

	PKTSETQCALLER_LIST(list_q->head, list_q->n_pkts, spq, CALL_SITE);

	/* empty list check */
	if (list_q->head == NULL)
		goto done;

	ASSERT_FP(PKTLINK(list_q->tail) == NULL);         /* terminated list */

	ASSERT_FP(!spktq_full(spq));

	q = &spq->q;

	if (q->head)
		PKTSETLINK(q->tail, list_q->head);
	else
		q->head = list_q->head;

	q->tail = list_q->tail;
	q->n_pkts += list_q->n_pkts;

#ifdef WL_TXQ_STALL
	list_q->dequeue_count += list_q->n_pkts;
#endif

	list_q->head = NULL;
	list_q->tail = NULL;
	list_q->n_pkts = 0;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return;
}

/*
 * Prepend spktq 'list' to the head of pktq 'pq'
 */
void
BCMFASTPATH(pktq_prepend)(struct pktq *pq, int prec, struct spktq *list)
{
	struct pktq_prec *q;
	struct pktq_prec *list_q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	list_q = &list->q;

	PKTSETQCALLER_LIST(list_q->head, list_q->n_pkts, pq, CALL_SITE);

	/* empty list check */
	if (list_q->head == NULL)
		goto done;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);
	ASSERT_FP(PKTLINK(list_q->tail) == NULL);         /* terminated list */

	ASSERT_FP(!pktq_full(pq));
	ASSERT_FP(!pktqprec_full(pq, prec));

	q = &pq->q[prec];

	/* set the tail packet of list to point at the former pq head */
	PKTSETLINK(list_q->tail, q->head);
	/* the new q head is the head of list */
	q->head = list_q->head;

	/* If the q tail was non-null, then it stays as is.
	 * If the q tail was null, it is now the tail of list
	 */
	if (q->tail == NULL) {
		q->tail = list_q->tail;
	}

	q->n_pkts += list_q->n_pkts;
	pq->n_pkts_tot += list_q->n_pkts;

	if (pq->hi_prec < prec)
		pq->hi_prec = (uint8)prec;

#ifdef WL_TXQ_STALL
	list_q->dequeue_count += list_q->n_pkts;
#endif

	list_q->head = NULL;
	list_q->tail = NULL;
	list_q->n_pkts = 0;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return;
}

/*
 * Prepend spktq 'list' to the head of spktq 'spq'
 */
void
BCMFASTPATH(spktq_prepend)(struct spktq *spq, struct spktq *list)
{
	struct pktq_prec *q;
	struct pktq_prec *list_q;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	list_q = &list->q;

	PKTSETQCALLER_LIST(list_q->head, list_q->n_pkts, spq, CALL_SITE);

	/* empty list check */
	if (list_q->head == NULL)
		goto done;

	ASSERT_FP(PKTLINK(list_q->tail) == NULL);         /* terminated list */

	ASSERT_FP(!spktq_full(spq));

	q = &spq->q;

	/* set the tail packet of list to point at the former pq head */
	PKTSETLINK(list_q->tail, q->head);
	/* the new q head is the head of list */
	q->head = list_q->head;

	/* If the q tail was non-null, then it stays as is.
	 * If the q tail was null, it is now the tail of list
	 */
	if (q->tail == NULL) {
		q->tail = list_q->tail;
	}

	q->n_pkts += list_q->n_pkts;

#ifdef WL_TXQ_STALL
	list_q->dequeue_count += list_q->n_pkts;
#endif

	list_q->head = NULL;
	list_q->tail = NULL;
	list_q->n_pkts = 0;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return;
}

void *
BCMFASTPATH(pktq_pdeq_prev)(struct pktq *pq, int prec, void *prev_p)
{
	struct pktq_prec *q;
	void *p = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);

	q = &pq->q[prec];

	if (prev_p == NULL)
		goto done;

	if ((p = PKTLINK(prev_p)) == NULL)
		goto done;

	q->n_pkts--;

	pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif
	PKTSETLINK(prev_p, PKTLINK(p));
	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, pq, CALL_SITE);
	return p;
}

void *
BCMFASTPATH(pktq_pdeq_with_fn)(struct pktq *pq, int prec, ifpkt_cb_t fn, int arg)
{
	struct pktq_prec *q;
	void *p, *prev = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);

	q = &pq->q[prec];
	p = q->head;

	while (p) {
		if (fn == NULL || (*fn)(p, arg)) {
			break;
		} else {
			prev = p;
			p = PKTLINK(p);
		}
	}
	if (p == NULL)
		goto done;

	if (prev == NULL) {
		if ((q->head = PKTLINK(p)) == NULL) {
			q->tail = NULL;
		}
	} else {
		PKTSETLINK(prev, PKTLINK(p));
		if (q->tail == p) {
			q->tail = prev;
		}
	}

	q->n_pkts--;

	pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif
	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, pq, CALL_SITE);
	return p;
}

bool
BCMFASTPATH(pktq_pdel)(struct pktq *pq, void *pktbuf, int prec)
{
	bool ret = FALSE;
	struct pktq_prec *q;
	void *p = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return FALSE;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);

	/* Should this just assert pktbuf? */
	if (!pktbuf)
		goto done;

	q = &pq->q[prec];

	if (q->head == pktbuf) {
		if ((q->head = PKTLINK(pktbuf)) == NULL)
			q->tail = NULL;
	} else {
		for (p = q->head; p && PKTLINK(p) != pktbuf; p = PKTLINK(p))
			;
		if (p == NULL)
			goto done;

		PKTSETLINK(p, PKTLINK(pktbuf));
		if (q->tail == pktbuf)
			q->tail = p;
	}

	q->n_pkts--;
	pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif

	PKTSETLINK(pktbuf, NULL);
	ret = TRUE;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	return ret;
}

static void
_pktq_pfilter(struct pktq *pq, int prec, pktq_filter_t fltr, void* fltr_ctx,
              defer_free_pkt_fn_t defer, void *defer_ctx)
{
	struct pktq_prec wq;
	struct pktq_prec *q;
	void *p;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	/* move the prec queue aside to a work queue */
	q = &pq->q[prec];

	wq = *q;

	q->head = NULL;
	q->tail = NULL;
	q->n_pkts = 0;

#ifdef WL_TXQ_STALL
	q->dequeue_count += wq.n_pkts;
#endif

	pq->n_pkts_tot -= wq.n_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return;

	/* start with the head of the work queue */
	while ((p = wq.head) != NULL) {
		/* unlink the current packet from the list */
		wq.head = PKTLINK(p);
		PKTSETLINK(p, NULL);
		wq.n_pkts--;

#ifdef WL_TXQ_STALL
		wq.dequeue_count++;
#endif

		/* call the filter function on current packet */
		ASSERT(fltr != NULL);
		switch ((*fltr)(fltr_ctx, p)) {
		case PKT_FILTER_NOACTION:
			/* put this packet back */
			pktq_penq(pq, prec, p);
			break;

		case PKT_FILTER_DELETE:
			/* delete this packet */
			ASSERT(defer != NULL);
			(*defer)(defer_ctx, p);
			break;

		case PKT_FILTER_REMOVE:
			/* pkt already removed from list */
			break;

		default:
			ASSERT(0);
			break;
		}
	}

	ASSERT(wq.n_pkts == 0);
}

void
pktq_pfilter(struct pktq *pq, int prec, pktq_filter_t fltr, void* fltr_ctx,
	defer_free_pkt_fn_t defer, void *defer_ctx, flush_free_pkt_fn_t flush, void *flush_ctx)
{
	_pktq_pfilter(pq, prec, fltr, fltr_ctx, defer, defer_ctx);

	ASSERT(flush != NULL);
	(*flush)(flush_ctx);
}

void
pktq_filter(struct pktq *pq, pktq_filter_t fltr, void* fltr_ctx,
	defer_free_pkt_fn_t defer, void *defer_ctx, flush_free_pkt_fn_t flush, void *flush_ctx)
{
	bool filter = FALSE;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	/* Optimize if pktq n_pkts = 0, just return.
	 * pktq len of 0 means pktq's prec q's are all empty.
	 */
	if (pq->n_pkts_tot > 0) {
		filter = TRUE;
	}

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return;

	if (filter) {
		int prec;

		PKTQ_PREC_ITER(pq, prec) {
			_pktq_pfilter(pq, prec, fltr, fltr_ctx, defer, defer_ctx);
		}

		ASSERT(flush != NULL);
		(*flush)(flush_ctx);
	}
}

void
spktq_filter(struct spktq *spq, pktq_filter_t fltr, void* fltr_ctx,
	defer_free_pkt_fn_t defer, void *defer_ctx, flush_free_pkt_fn_t flush, void *flush_ctx)
{
	struct pktq_prec wq;
	struct pktq_prec *q;
	void *p = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	q = &spq->q;

	/* Optimize if pktq_prec n_pkts = 0, just return. */
	if (q->n_pkts == 0) {
		(void)HND_PKTQ_MUTEX_RELEASE(&spq->mutex);
		return;
	}

	wq = *q;

	q->head = NULL;
	q->tail = NULL;
	q->n_pkts = 0;

#ifdef WL_TXQ_STALL
	q->dequeue_count += wq.n_pkts;
#endif

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return;

	/* start with the head of the work queue */

	while ((p = wq.head) != NULL) {
		/* unlink the current packet from the list */
		wq.head = PKTLINK(p);
		PKTSETLINK(p, NULL);
		wq.n_pkts--;

#ifdef WL_TXQ_STALL
		wq.dequeue_count++;
#endif

		/* call the filter function on current packet */
		ASSERT(fltr != NULL);
		switch ((*fltr)(fltr_ctx, p)) {
		case PKT_FILTER_NOACTION:
			/* put this packet back */
			spktq_enq(spq, p);
			break;

		case PKT_FILTER_DELETE:
			/* delete this packet */
			ASSERT(defer != NULL);
			(*defer)(defer_ctx, p);
			break;

		case PKT_FILTER_REMOVE:
			/* pkt already removed from list */
			break;

		default:
			ASSERT(0);
			break;
		}
	}

	ASSERT(wq.n_pkts == 0);

	ASSERT(flush != NULL);
	(*flush)(flush_ctx);
}

bool
pktq_init(struct pktq *pq, uint num_prec, uint max_pkts)
{
	uint prec;

	ASSERT_FP(num_prec > 0 && num_prec <= PKTQ_MAX_PREC);

	/* pq is variable size; only zero out what's requested */
	bzero(pq, OFFSETOF(struct pktq, q) + (sizeof(struct pktq_prec) * num_prec));

	if (HND_PKTQ_MUTEX_CREATE("pktq", &pq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	pq->num_prec = (uint16)num_prec;

	pq->max_pkts = (uint16)max_pkts;

	for (prec = 0; prec < num_prec; prec++)
		pq->q[prec].max_pkts = pq->max_pkts;

	return TRUE;
}

bool
spktq_init(struct spktq *spq, uint max_pkts)
{
	bzero(spq, sizeof(struct spktq));

	if (HND_PKTQ_MUTEX_CREATE("spktq", &spq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	spq->q.max_pkts = (uint16)max_pkts;

	return TRUE;
}

bool
BCMFASTPATH(spktq_init_list)(struct spktq *spq, uint max_pkts, void *head,
	void *tail, uint16 n_pkts)
{
	if (HND_PKTQ_MUTEX_CREATE("spktq", &spq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	ASSERT_FP(PKTLINK(tail) == NULL);
	PKTSETLINK(tail, NULL);
	spq->q.head = head;
	spq->q.tail = tail;
	spq->q.max_pkts = (uint16)max_pkts;
	spq->q.n_pkts = n_pkts;
	spq->q.stall_count = 0;
	spq->q.dequeue_count = 0;

	return TRUE;
}

bool
pktq_deinit(struct pktq *pq)
{
	BCM_REFERENCE(pq);
	if (HND_PKTQ_MUTEX_DELETE(&pq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	return TRUE;
}

bool
spktq_deinit(struct spktq *spq)
{
	BCM_REFERENCE(spq);
	if (HND_PKTQ_MUTEX_DELETE(&spq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	return TRUE;
}

void
pktq_set_max_plen(struct pktq *pq, int prec, uint max_pkts)
{
	ASSERT(prec >= 0 && prec < pq->num_prec);

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	if (prec < pq->num_prec)
		pq->q[prec].max_pkts = (uint16)max_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return;
}

void *
BCMFASTPATH(pktq_deq)(struct pktq *pq, int *prec_out)
{
	struct pktq_prec *q;
	void *p = NULL;
	int prec;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	if (pq->n_pkts_tot == 0)
		goto done;

	while ((prec = pq->hi_prec) > 0 && pq->q[prec].head == NULL)
		pq->hi_prec--;

	q = &pq->q[prec];

	if ((p = q->head) == NULL)
		goto done;

	if ((q->head = PKTLINK(p)) == NULL)
		q->tail = NULL;

	q->n_pkts--;

	pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif

	if (prec_out)
		*prec_out = prec;

	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, pq, CALL_SITE);
	return p;
}

void *
BCMFASTPATH(pktq_deq_tail)(struct pktq *pq, int *prec_out)
{
	struct pktq_prec *q;
	void *p = NULL, *prev;
	int prec;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	if (pq->n_pkts_tot == 0)
		goto done;

	for (prec = 0; prec < pq->hi_prec; prec++)
		if (pq->q[prec].head)
			break;

	q = &pq->q[prec];

	if ((p = q->head) == NULL)
		goto done;

	for (prev = NULL; p != q->tail; p = PKTLINK(p))
		prev = p;

	if (prev)
		PKTSETLINK(prev, NULL);
	else
		q->head = NULL;

	q->tail = prev;
	q->n_pkts--;

	pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif

	if (prec_out)
		*prec_out = prec;

	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, pq, CALL_SITE);
	return p;
}

void *
BCMPOSTTRAPFN(pktq_peek)(struct pktq *pq, int *prec_out)
{
	int prec;
	void *p = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	if (pq->n_pkts_tot == 0)
		goto done;

	while ((prec = pq->hi_prec) > 0 && pq->q[prec].head == NULL)
		pq->hi_prec--;

	if (prec_out)
		*prec_out = prec;

	p = pq->q[prec].head;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

void *
spktq_peek(struct spktq *spq)
{
	void *p = NULL;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	if (spq->q.n_pkts == 0)
		goto done;

	p = spq->q.head;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

void
pktq_pflush(osl_t *osh, struct pktq *pq, int prec, bool dir)
{
	void *p;

	/* no need for a mutex protection! */

	/* start with the head of the list */
	while ((p = pktq_pdeq(pq, prec)) != NULL) {

		/* delete this packet */
		PKTFREE(osh, p, dir);
	}
}

void
spktq_flush_ext(osl_t *osh, struct spktq *spq, bool dir,
	void (*pktq_flush_cb)(void *ctx, void *pkt), void *pktq_flush_ctx)
{
	void *pkt;

	/* no need for a mutex protection! */

	/* start with the head of the list */
	while ((pkt = spktq_deq(spq)) != NULL) {
		if (pktq_flush_cb != NULL) {
			pktq_flush_cb(pktq_flush_ctx, pkt);
		}
		/* delete this packet */
		PKTFREE(osh, pkt, dir);
	}
}

typedef struct {
	spktq_cb_t cb;
	void *arg;
} spktq_cbinfo_t;

typedef struct {
	spktq_suppress_cb_t cb;
	void *arg;
} spktq_suppress_cbinfo_t;
static spktq_cbinfo_t spktq_cbinfo = {NULL, NULL};
static spktq_cbinfo_t *spktq_cbinfo_get(void);

static spktq_suppress_cbinfo_t spktq_suppress_cbinfo = {NULL, NULL};
static spktq_suppress_cbinfo_t *spktq_suppress_cbinfo_get(void);

/* Accessor function forced into RAM to keep spktq_cbinfo out of shdat */
static spktq_cbinfo_t*
BCMRAMFN(spktq_cbinfo_get)(void)
{
	return (&spktq_cbinfo);
}

static spktq_suppress_cbinfo_t*
BCMRAMFN(spktq_suppress_cbinfo_get)(void)
{
	return (&spktq_suppress_cbinfo);
}

void
spktq_free_register(spktq_cb_t cb, void *arg)
{
	spktq_cbinfo_t *cbinfop = spktq_cbinfo_get();
	cbinfop->cb = cb;
	cbinfop->arg = arg;
}

void
spktq_cb(void *spq)
{
	spktq_cbinfo_t *cbinfop = spktq_cbinfo_get();
	if (cbinfop->cb) {
		cbinfop->cb(cbinfop->arg, spq);
	}
}

void
spktq_suppress_register(spktq_suppress_cb_t cb, void *arg)
{
	spktq_suppress_cbinfo_t *cbinfop = spktq_suppress_cbinfo_get();
	cbinfop->cb = cb;
	cbinfop->arg = arg;
}

uint32
spktq_suppress_cb(void *spq)
{
	spktq_suppress_cbinfo_t *cbinfop = spktq_suppress_cbinfo_get();
	if (cbinfop->cb) {
		return cbinfop->cb(cbinfop->arg, spq);
	}
	return BCME_UNSUPPORTED;
}

void
pktq_flush(osl_t *osh, struct pktq *pq, bool dir)
{
	bool flush = FALSE;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return;

	/* Optimize flush, if pktq n_pkts_tot = 0, just return.
	 * pktq len of 0 means pktq's prec q's are all empty.
	 */
	if (pq->n_pkts_tot > 0) {
		flush = TRUE;
	}

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return;

	if (flush) {
		int prec;

		PKTQ_PREC_ITER(pq, prec) {
			pktq_pflush(osh, pq, prec, dir);
		}
	}
}

/* Return sum of lengths of a specific set of precedences */
int
pktq_mlen(struct pktq *pq, uint prec_bmp)
{
	int prec, len;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return 0;

	len = 0;

	for (prec = 0; prec <= pq->hi_prec; prec++)
		if (prec_bmp & (1u << prec))
			len += pq->q[prec].n_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return 0;

	return len;
}

/* Priority peek from a specific set of precedences */
void *
BCMPOSTTRAPFASTPATH(pktq_mpeek)(struct pktq *pq, uint prec_bmp, int *prec_out)
{
	struct pktq_prec *q;
	void *p = NULL;
	int prec;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	if (pq->n_pkts_tot == 0)
		goto done;

	while ((prec = pq->hi_prec) > 0 && pq->q[prec].head == NULL)
		pq->hi_prec--;

	while ((prec_bmp & (1u << prec)) == 0 || pq->q[prec].head == NULL)
		if (prec-- == 0)
			goto done;

	q = &pq->q[prec];

	if ((p = q->head) == NULL)
		goto done;

	if (prec_out)
		*prec_out = prec;

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	return p;
}

/* Priority dequeue from a specific set of precedences */
void *
BCMPOSTTRAPFASTPATH(pktq_mdeq)(struct pktq *pq, uint prec_bmp, int *prec_out)
{
	struct pktq_prec *q;
	void *p = NULL;
	int prec;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return NULL;

	if (pq->n_pkts_tot == 0)
		goto done;

	while ((prec = pq->hi_prec) > 0 && pq->q[prec].head == NULL)
		pq->hi_prec--;

	while ((pq->q[prec].head == NULL) || ((prec_bmp & (1u << prec)) == 0))
		if (prec-- == 0)
			goto done;

	q = &pq->q[prec];

	if ((p = q->head) == NULL)
		goto done;

	if ((q->head = PKTLINK(p)) == NULL)
		q->tail = NULL;

	q->n_pkts--;

#ifdef WL_TXQ_STALL
	q->dequeue_count++;
#endif

	if (prec_out)
		*prec_out = prec;

	pq->n_pkts_tot--;

	PKTSETLINK(p, NULL);

done:
	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return NULL;

	PKTSETQCALLER(p, pq, CALL_SITE);
	return p;
}

#ifdef HND_PKTQ_THREAD_SAFE
int
pktqprec_avail_pkts(struct pktq *pq, int prec)
{
	int ret;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return 0;

	ASSERT(prec >= 0 && prec < pq->num_prec);

	ret = pq->q[prec].max_pkts - pq->q[prec].n_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return 0;

	return ret;
}

bool
BCMFASTPATH(pktqprec_full)(struct pktq *pq, int prec)
{
	bool ret;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return FALSE;

	ASSERT_FP(prec >= 0 && prec < pq->num_prec);

	ret = pq->q[prec].n_pkts >= pq->q[prec].max_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	return ret;
}

int
pktq_avail(struct pktq *pq)
{
	int ret;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return 0;

	ret = pq->max_pkts - pq->n_pkts_tot;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return 0;

	return ret;
}

int
spktq_avail(struct spktq *spq)
{
	int ret;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return 0;

	ret = spq->q.max_pkts - spq->q.n_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return 0;

	return ret;
}

bool
pktq_full(struct pktq *pq)
{
	bool ret;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&pq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return FALSE;

	ret = pq->n_pkts_tot >= pq->max_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&pq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	return ret;
}

bool
spktq_full(struct spktq *spq)
{
	bool ret;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_ACQUIRE(&spq->mutex, OSL_EXT_TIME_FOREVER) != OSL_EXT_SUCCESS)
		return FALSE;

	ret = spq->q.n_pkts >= spq->q.max_pkts;

	/* protect shared resource */
	if (HND_PKTQ_MUTEX_RELEASE(&spq->mutex) != OSL_EXT_SUCCESS)
		return FALSE;

	return ret;
}

#endif	/* HND_PKTQ_THREAD_SAFE */
