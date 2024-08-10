
#include "fetchunit_types.h"
#include "btb.h"
#include "bq.h"
#include "gshare.h"
#include "ras.h"
#include "perfectbp.h"
#include "ic.h"
#include "tc.h"

// Forward declaring pipeline_t class.
class pipeline_t;

class fetchunit_t
{
private:
    // Fetch bundle constraints.
    uint64_t instr_per_cycle;
    uint64_t cond_branch_per_cycle;

    // The PAY buffer holds the payload of each in-flight instruction in the pipeline.
    payload *PAY;
    pipeline_t *proc; // This is needed by PAY->map_to_actual() and PAY->predict().

    ////////////////////////////////////////////////////////////////
    // Fetch1 Stage.
    ////////////////////////////////////////////////////////////////

    // The Fetch1 stage is active unless it is waiting for a serializing instruction (fetch exception, amo, or csr instruction) to retire.
    bool fetch_active;

    // Start PC of the fetch bundle in the Fetch1 stage.
    uint64_t pc;

    // The fetch bundle from the instruction cache + BTB or from the trace cache.
    fetch_bundle_t *fetch_bundle;

    // Instruction Cache
    ic_t ic;
    bool ic_miss;
    cycle_t ic_miss_resolve_cycle;

    // Branch Target Buffer (BTB):
    //
    // Locates branches within a sequential fetch bundle, and provides their types and
    // taken targets (latter for just conditional branches and direct jumps).
    //
    // The BTB's information, along with conditional branch T/NT predictions, is critical
    // for determining the fetch bundle's length and for selecting the PC of the next
    // fetch bundle among multiple choices.
    btb_t btb;

    // Trace Cache
    //
    // The trace selection policy MUST be as follows; you may add additional constraints, e.g., related
    // to the kinds of embedded conditional branches allowed (taken, confident, atomic trace sel., etc.).
    // - Stop after m'th conditional branch.
    // - Note: there can be any number of jump directs in the trace.
    // - Stop after call direct.    (Support one RAS operation per fetch cycle.)
    // - Stop after jump indirect.  (Don't want indirect targets to be part of the trace cache hit logic.)
    // - Stop after call indirect.  (Don't want indirect targets to be part of the trace cache hit logic.
    //                                  Support one RAS operation per fetch cycle.)
    // - Stop after return.         (Don't want indirect targets to be part of the trace cache hit logic.
    //                                  Support one RAS operation per fetch cycle.)
    bool tc_enable;
    tc_t tc;

    // Gshare predictor for conditional branches.
    uint64_t *cb;
    gshare_index_t cb_index;

    // Gshare predictor for indirect branches.
    uint64_t *ib;
    gshare_index_t ib_index;

    // Return address stack for predicting return targets.
    ras_t ras;

    // Perfect branch predictor. Note: PAY->predict() serves as the perfect branch predictor.
    bool bp_perfect;

    ////////////////////////////////////////////////////////////////
    // Fetch2 Stage.
    ////////////////////////////////////////////////////////////////

    // Pipeline register between the Fetch1 and Fetch2 stages.
    pipeline_register *FETCH2;

    // Information about the fetch bundle in the Fetch2 stage.
    fetch2_status_t fetch2_status;

    // Branch queue for keeping track of all outstanding branch predictions.
    bq_t bq;

    // Measurements.
    uint64_t meas_branch_n;  // # branches
    uint64_t meas_jumpdir_n; // # jumps, direct
    uint64_t meas_calldir_n; // # calls, direct
    uint64_t meas_jumpind_n; // # jumps, indirect
    uint64_t meas_callind_n; // # calls, indirect
    uint64_t meas_jumpret_n; // # jumps, return

    uint64_t meas_branch_m;  // # mispredicted branches
    uint64_t meas_jumpind_m; // # mispredicted jumps, indirect
    uint64_t meas_callind_m; // # mispredicted calls, indirect
    uint64_t meas_jumpret_m; // # mispredicted jumps, return

    uint64_t meas_jumpind_seq; // # jump-indirect instructions whose targets were the next sequential PC

    uint64_t meas_btbmiss; // # of btb misses, i.e., number of discarded fetch bundles (idle fetch cycles) due to a btb miss within the bundle

    ////////////////////////////
    // Private functions.
    ////////////////////////////

    // Function for speculatively updating the pc, BHRs, and RAS, based on the assembled fetch bundle.
    void spec_update(spec_update_t *update, uint64_t cb_predictions);

    // Function for transferring the fetch bundle into (1) the PAY buffer and (2) the FETCH2 pipeline register.
    void transfer_fetch_bundle();

    // Function for squashing the Fetch2 stage, i.e., invalidate all instructions in the FETCH2 pipeline register and reset fetch2_status.
    void squash_fetch2();

public:
    fetchunit_t(uint64_t instr_per_cycle,                      // "n"
                uint64_t cond_branch_per_cycle,                // "m"
                uint64_t btb_entries,                          // total number of entries in the BTB
                uint64_t btb_assoc,                            // set-associativity of the BTB
                uint64_t cb_pc_length, uint64_t cb_bhr_length, // gshare cond. br. predictor: pc length (index size), bhr length
                uint64_t ib_pc_length, uint64_t ib_bhr_length, // gshare indirect br. predictor: pc length (index size), bhr length
                uint64_t ras_size,                             // # entries in the RAS
                uint64_t bq_size,                              // branch queue size (max. number of outstanding branches)
                bool tc_enable,                                // enable trace cache
                bool tc_perfect,                               // perfect trace cache (only relevant if trace cache is enabled)
                bool bp_perfect,                               // perfect branch prediction
                bool ic_perfect,                               // perfect instruction cache
                uint64_t ic_sets,                              // I$ sets
                uint64_t ic_assoc,                             // I$ set-associativity
                uint64_t ic_line_size,                         // log2(I$ line size in bytes): must be consistent with instr_per_cycle (see ic.h/cc).
                uint64_t ic_hit_latency,                       // I$ hit latency (overridden by Fetch1 stage being 1 cycle deep)
                uint64_t ic_miss_latency,                      // I$ miss latency (overridden by L2$ hit latency, only if L2$ exists)
                uint64_t ic_num_MHSRs,                         // I$ number of MHSRs
                uint64_t ic_miss_srv_ports,                    // see CacheClass.h/cc
                uint64_t ic_miss_srv_latency,                  // see CacheClass.h/cc
                CacheClass *L2C,                               // The L2 cache that backs the instruction cache.
                mmu_t *mmu,                                    // mmu is needed by (1) the instruction cache and (2) perfect trace cache mode
                pipeline_t *proc,                              // proc is needed by (1) PAY->map_to_actual() and PAY->predict(), and
                                                               //                   (2) instruction cache's access to proc's stats
                payload *PAY);                                 // (1) Payload of fetched instructions. (2) Provides a function that serves as a perfect branch predictor.
    ~fetchunit_t();

    // Predict and supply a fetch bundle from either the instruction cache + BTB or the trace cache.
    // The fetch bundle is placed in the FETCH2 pipeline register that separates the Fetch1 and Fetch2 stages.
    // Checkpoint (in fetch2_status) and then speculatively update the Fetch1 stage's pc, BHRs, etc., to set up for the next fetch cycle.
    // The caller of fetch1() passes in the current cycle so that the Fetch1 stage can model the cycle at which an instruction cache miss resolves.
    void fetch1(cycle_t cycle);

    // Fetch2 pipeline stage.
    // If it returns true: call fetchunit_t::fetch1() after.
    // If it returns false: do NOT call fetchunit_t::fetch1() after, because of a misfetch recovery.
    //
    // fetchunit_t::fetch2() performs two steps.
    // - Step 1: It predecodes the fetch bundle to detect a "misfetch" and identify serializing instructions (amo, csr, exception).
    //   If a misfetched bundle is detected (the fetch bundle was supplied by the instruction cache and some branches were missed by the BTB),
    //   recovery is performed (discard the misfetched bundle in Fetch2, train the BTB for all missed branches, restore Fetch1's state to set up
    //   repredicting the bundle, and inform the caller to not call fetchunit_t::fetch1() in this cycle).
    //   If a serializing instruction is detected, we ensure the bundle ends at the serializing instruction and make Fetch1 idle until
    //   the serializing instruction retires.
    // - Step 2: This step is skipped if there was a misfetch.  Otherwise, the fetch bundle is transferred from the FETCH2 pipeline register to
    //   the DECODE pipeline register, and all branches in the fetch bundle are pushed onto the branch queue (with their correct contexts for training
    //   the branch predictors when they commit).
    //
    // The caller passes in the DECODE pipeline register so that this function can transfer the fetch bundle to it.
    bool fetch2(pipeline_register DECODE[]);

    // A mispredicted branch was detected.
    // 1. Roll-back the branch queue to the mispredicted branch's entry.
    // 2. Correct the mispredicted branch's information in its branch queue entry.
    // 3. Restore checkpointed global histories and the RAS (as best we can for RAS).
    // 4. Note that the branch was mispredicted (for measuring mispredictions at retirement).
    // 5. Restore the pc.
    // 6. Go active again, whether or not currently active (restore fetch_active).
    // 7. Squash the fetch2_status register and FETCH2 pipeline register.
    void mispredict(uint64_t branch_pred_tag, bool taken, uint64_t next_pc);

    // Commit the indicated branch from the branch queue.
    // We assert that it is at the head.
    // void commit(uint64_t branch_pred_tag);
    void commit();

    // Complete squash.
    // 1. Roll-back the branch queue to the head entry.
    // 2. Restore checkpointed global histories and the RAS (as best we can for RAS).
    // 3. Restore the pc.
    // 4. Go active again, whether or not currently active (restore fetch_active).
    // 5. Squash the fetch2_status register and FETCH2 pipeline register.
    // 6. Reset ic_miss (discard pending I$ misses).
    void flush(uint64_t pc);

    // Output all branch prediction measurements.
    void output(uint64_t num_instr, uint64_t num_cycles, FILE *fp);

    // Public functions for setting and getting the speculative pc directly.
    void setPC(uint64_t pc);
    uint64_t getPC();

    // Public function for querying fetch_active.
    bool active();
};
