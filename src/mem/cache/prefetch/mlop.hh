/**
 * Copyright (c) 2025 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2018 Metempsy Technology Consulting
 * Copyright (c) 2024 Samsung Electronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Implementation of the 'Multiple Lookahead Offset Prefetcher'
 * Reference:
 *   Mehran Shakerinava, Mohammad Bakhshalipour, Pejman Lotfi-Kamran, Hamid
 *  Sarbazi-Azad (2019)
 *  In 3rd Data Prefetching Championship.
 *
 * Implementation based upon BOP, see bop.cc, bop.hh
 */

#ifndef __MEM_CACHE_PREFETCH_MLOP_HH__
#define __MEM_CACHE_PREFETCH_MLOP_HH__

#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct MLOPPrefetcherParams;

namespace prefetch
{

class MLOP : public Queued
{
  private:
    enum RRWay
    {
        Left,
        Right
    };

    /** Learning phase parameters */
    const unsigned int scoreMax;
    const unsigned int roundMax;
    const unsigned int badScore;
    /** Recent requests table parameteres */
    const unsigned int rrEntries;
    const unsigned int tagMask;
    /** Delay queue parameters */
    const bool delayQueueEnabled;
    const unsigned int delayQueueSize;
    const unsigned int delayTicks;

    std::vector<Addr> rrLeft;
    std::vector<Addr> rrRight;

    /** Structure to save the offset and the score */
    typedef std::pair<int16_t, uint8_t> OffsetListEntry;
    std::vector<std::vector<OffsetListEntry>> offsetsList;
    std::vector<uint8_t> phaseDegreeBestOffset;
    uint8_t currentOffsetIdx;

    /** Best offsets to use for actual prefetching */
    std::vector<uint8_t> shouldPrefetch;
    std::vector<uint8_t> degreeBestOffset;

    unsigned int bestScore;

    /** In a first implementation of the BO prefetcher, both banks of the
     *  RR were written simultaneously when a prefetched line is inserted
     *  into the cache. Adding the delay queue tries to avoid always
     *  striving for timeless prefetches, which has been found to not
     *  always being optimal.
     */
    struct DelayQueueEntry
    {
        Addr baseAddr;
        Tick processTick;

        DelayQueueEntry(Addr x, Tick t) : baseAddr(x), processTick(t) {}
    };

    std::deque<DelayQueueEntry> delayQueue;

    /** Event to handle the delay queue processing */
    void delayQueueEventWrapper();
    EventFunctionWrapper delayQueueEvent;

    /** Current round */
    unsigned int round;

    /** Generate a hash for the specified address to index the RR table
     *  @param addr: address to hash
     *  @param way:  RR table to which is addressed (left/right)
     */
    unsigned int index(Addr addr, unsigned int way) const;

    /** Insert the specified address into the RR table
     *  @param addr: address to insert
     *  @param addr_tag: The tag to insert in the RR table
     *  @param way: RR table to which the address will be inserted
     */
    void insertIntoRR(Addr addr, Addr addr_tag, unsigned int way);

    /** Insert the specified address into the delay queue. This will
     *  trigger an event after the delay cycles pass
     *  @param addr: address to insert into the delay queue
     */
    void insertIntoDelayQueue(Addr addr);

    /** Reset all the scores from the offset list */
    void resetScores();

    /** Generate the tag for the specified address based on the tag bits
     *  and the block size
     *  @param addr: address to get the tag from
     */
    Addr tag(Addr addr) const;

    /** Test if @X-O is hitting in the RR table to update the
     *  offset score
     *  @param addr_tag: tag searched for within the RR
     */
    bool testRR(Addr addr_tag) const;

    /** Learning phase of the MLOP. Update the intermediate values of the
     * round and update the best offset if found
     * @param addr: full address used to compute X-O tag to determine
     *              offset efficacy.
     */
    void bestOffsetLearning(Addr addr);

    /** Update the RR right table after a prefetch fill */
    void notifyFill(const CacheAccessProbeArg &arg) override;

  public:
    /** The prefetch degree, i.e. the number of prefetches to generate */
    unsigned int degree;

    MLOP(const MLOPPrefetcherParams &p);
    ~MLOP() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif /* __MEM_CACHE_PREFETCH_MLOP_HH__ */
