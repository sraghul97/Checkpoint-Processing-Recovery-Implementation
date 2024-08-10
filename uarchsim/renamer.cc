#include "renamer.h"
#include <assert.h>
#include <inttypes.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <vector>
using namespace std;

// Renamer Constructor
renamer::renamer(uint64_t n_log_regs, uint64_t n_phys_regs,
                 uint64_t n_chkpoints, uint64_t rob_size) {
  // Assert the number of physical registers > number logical registers.
  assert(n_phys_regs > n_log_regs);
  // cout << "Assert the number of physical registers > number logical
  // registers";
  //  Assert 1 <= n_branches <= 64.
  assert(n_chkpoints >= 1 && n_chkpoints <= 64);
  // cout << "Assert 1 <= n_branches <= 64";
  //  Assert n_active > 0.
  assert(rob_size > 0);
  // cout << "Assert n_active > 0";
  //  Then, allocate space for the primary data structures.
  //  Then, initialize the data structures based on the knowledge
  //  that the pipeline is intially empty (no in-flight instructions yet).

  // Initializing RMT Structure
  for (uint64_t i = 0; i < n_log_regs; i++) {
    RMT.push_back({i, i});
  }
  // cout << "RMT Initialized";

  // Initializing AMT Structure
  //  for (uint64_t j = 0; j < n_log_regs; j++) {
  //      AMT.push_back({j,j});
  //  }
  // cout << "AMT Initialized";

  // Initializing FreeList Structure
  RMTSize = n_log_regs;
  // AMTSize = n_log_regs;
  PRFSize = n_phys_regs;
  // ActiveListSize = n_active;
  FreeListSize = n_phys_regs - n_log_regs;
  CheckpointBufferSize = n_chkpoints;
  max_instr_bw_checkpoints = rob_size / n_chkpoints;
  // BranchCheckPointsSize = n_branches;
  // int FreeListSize = n_phys_regs - n_log_regs;
  uint64_t temp = n_log_regs;
  FL.FreeListSize = FreeListSize;
  FL.Head = 0;
  FL.Tail = 0;
  // Filling up the FreeList and hence the Head Phase and tail Phase pointers
  // are not equal
  FL.HeadPhaseBit = 1;
  FL.TailPhaseBit = 0;
  for (uint64_t k = 0; k < FreeListSize; k++) {
    FL.FList.push_back({temp});
    temp++;
  }
  // cout << "FreeList Initialized";

  // Initializing ActiveList Structure
  // AL.ActiveListSize = ActiveListSize;
  // AL.Head = 0;
  //  AL.Tail = 0;
  //  AL.HeadPhaseBit = 0;
  //  AL.TailPhaseBit = 0;
  //  //Initially there are no inflight instructions so Active list is empty
  //  for (uint64_t l = 0; l < ActiveListSize; l++) {
  //      AL.AL_entries.push_back({0,0,0,0,0,0,0,0,0,0,0,0,0,0});
  //  }
  // cout << "ActiveList Initialized";

  // Initializing PhysicalRegisterFile Structure
  // Initially the RMT i.e, the first few registers i.e., the logical registers
  // are already checkpointed and hence
  //  their usage counter is 1 and unmapped bit is 0
  for (uint64_t m = 0; m < n_log_regs; m++) {
    PRF.push_back({m, (uint64_t)0, 0, 1});
  }
  for (uint64_t n = n_log_regs; n < PRFSize; n++) {
    PRF.push_back({n, (uint64_t)0, 1, 0});
  }
  // cout << "PhysicalRegister Initialized";

  // Initializing PRF_ReadyBitArray
  // Initially setting the logical registers ready bit as 1 and the remaining
  // ready bits to 0
  for (uint64_t n = 0; n < n_log_regs; n++) {
    PRF_ReadyBitArray.push_back({n, 1});
  }
  for (uint64_t p = n_log_regs; p < n_phys_regs; p++) {
    PRF_ReadyBitArray.push_back({p, 0});
  }
  // cout << "PRF Array Initialized";

  // Initializing GBM
  // GBM = 0;
  // cout << "GBM Initialized";

  // Initializing Checkpoint Buffer
  // Initially the first checkpoint(oldest checkpoint) will contain the RMT
  // during the initial state and the other checkpoints are not yet used
  for (uint64_t z = 0; z < CheckpointBufferSize; z++) {
    CheckpointBuffer.Checkpoint.push_back(Checkpoint_params());
    if (z == 0) {
      for (uint64_t y = 0; y < n_log_regs; y++) {
        CheckpointBuffer.Checkpoint[z].CheckpointedRMT.push_back(
            {y, (uint64_t)y});
      }
      for (uint64_t k = 0; k < n_log_regs; k++) {
        CheckpointBuffer.Checkpoint[z].CheckpointedUnmappedBits.push_back({0});
      }
      for (uint64_t k = n_log_regs; k < PRFSize; k++) {
        CheckpointBuffer.Checkpoint[z].CheckpointedUnmappedBits.push_back({1});
      }
    } else {
      for (uint64_t y = 0; y < n_log_regs; y++) {
        CheckpointBuffer.Checkpoint[z].CheckpointedRMT.push_back(
            {y, (uint64_t)0});
      }
      for (uint64_t k = 0; k < PRFSize; k++) {
        CheckpointBuffer.Checkpoint[z].CheckpointedUnmappedBits.push_back({1});
      }
    }
    CheckpointBuffer.Checkpoint[z].UncompletedInstructionCounter = 0;
    CheckpointBuffer.Checkpoint[z].LoadCounter = 0;
    CheckpointBuffer.Checkpoint[z].StoreCounter = 0;
    CheckpointBuffer.Checkpoint[z].BranchCounter = 0;
    CheckpointBuffer.Checkpoint[z].AMO_Flag = 0;
    CheckpointBuffer.Checkpoint[z].CSR_Flag = 0;
    CheckpointBuffer.Checkpoint[z].Exception_Flag = 0;
  }
  CheckpointBuffer.CheckpointBufferSize = CheckpointBufferSize;
  CheckpointBuffer.Head = 0;
  CheckpointBuffer.Tail = 1; // since the oldest checkpoint is already used
  CheckpointBuffer.HeadPhaseBit = 0;
  CheckpointBuffer.TailPhaseBit = 0;
  // Initializing Branch Checkpoints structure
  //  for (uint64_t q = 0; q < BranchCheckPointsSize; q++) {
  //      BranchCheckpoints.push_back(BranchCheckpoints_params());
  //      for (uint64_t x = 0; x < n_log_regs; x++) {
  //          BranchCheckpoints[q].CheckpointedRMT.push_back({x,(uint64_t)0});
  //      }
  //      BranchCheckpoints[q].CheckpointedFreeListHead = 0;
  //      BranchCheckpoints[q].CheckpointedFreeListHeadPhaseBit = 0;
  //      BranchCheckpoints[q].CheckpointedGBM = 0;
  //  }
  // cout << "Branch Checkpoints Initialized";
  // FL.vect[0]
}

// renamer destructor
renamer::~renamer() {}

bool renamer::stall_reg(uint64_t bundle_dst) {
  // Inputs: bundle_dst: number of logical destination registers in current
  // rename bundle Return value: Return "true" (stall) if there aren't enough
  // free physical registers to allocate to all of the logical destination
  // register in the current rename bundle

  // Finding the number of free physical registers to allocate to logical
  // registers
  uint64_t NumberOfFreePhysicalRegisters;
  if (FL.Head == FL.Tail && FL.HeadPhaseBit != FL.TailPhaseBit) {
    // the Free List is full and hence there are Free list size physical
    // registers that are ready to be allocated for the logical destination
    // register
    NumberOfFreePhysicalRegisters = FL.FreeListSize;
  } else if (FL.Head == FL.Tail && FL.HeadPhaseBit == FL.TailPhaseBit) {
    // the Free List is empty and hence there are no Free physical registers
    // that are ready to be allocated for the logical destionation register
    NumberOfFreePhysicalRegisters = 0;
  } else if (FL.Head != FL.Tail && FL.HeadPhaseBit != FL.TailPhaseBit) {
    NumberOfFreePhysicalRegisters = FL.FreeListSize - (FL.Head - FL.Tail);
  } else if (FL.Head != FL.Tail && FL.HeadPhaseBit == FL.TailPhaseBit) {
    NumberOfFreePhysicalRegisters = FL.Tail - FL.Head;
  }

  // if the number of free physical registers are greater than or equal to the
  // number of logical destination registers in the current rename bundle then
  // return false else true
  if (NumberOfFreePhysicalRegisters >= bundle_dst) {
    return false;
  } else {
    return true;
  }
}

// bool renamer::stall_branch(uint64_t bundle_branch) {
//     // Inputs: bundle_branch: number of branches in current rename bundle
// 	// Return value: Return "true" (stall) if there aren't enough free
// checkpoints for all branches in the current rename bundle.

//     //Finding the number of unset bits in the GBM to calculate the number of
//     free checkpoints uint64_t NumberOfSetBitsinGBM = 0; uint64_t temporary =
//     GBM; for (; temporary; NumberOfSetBitsinGBM++)
//         temporary &= temporary - (uint64_t)1;
//     uint64_t NumberOfUnsetBitsinGBM = BranchCheckPointsSize -
//     NumberOfSetBitsinGBM; if (NumberOfUnsetBitsinGBM >= bundle_branch) {
//         return false;
//     } else {
//         return true;
//     }
// }

void renamer::inc_usage_counter(uint64_t phys_reg) {
  // This function is used to increment the usage counter of a given physical
  // register
  for (uint64_t m = 0; m < PRFSize; m++) {
    if (PRF[m].PhysicalRegisterNumber == phys_reg) {
      PRF[m].UsageCounter++;
      break;
    }
  }
}

void renamer::dec_usage_counter(uint64_t phys_reg) {
  // This function is used to decrement the usage counter of a given physical
  // register
  for (uint64_t m = 0; m < PRFSize; m++) {
    if (PRF[m].PhysicalRegisterNumber == phys_reg) {
      assert(PRF[m].UsageCounter > 0);
      PRF[m].UsageCounter--;
      if ((PRF[m].UsageCounter == 0) && (PRF[m].UnmappedBit)) {
        FL.FList[FL.Tail] = phys_reg;
        FL.Tail++;
        if (FL.Tail == FL.FreeListSize) {
          FL.Tail = 0;
          FL.TailPhaseBit = !FL.TailPhaseBit;
        }
      }
      break;
    }
  }
}

void renamer::map(uint64_t phys_reg) { PRF[phys_reg].UnmappedBit = false; }

void renamer::unmap(uint64_t phys_reg) {
  if (!PRF[phys_reg].UnmappedBit) {
    PRF[phys_reg].UnmappedBit = true;
    if (PRF[phys_reg].UsageCounter == 0) {
      FL.FList[FL.Tail] = phys_reg;
      FL.Tail++;
      if (FL.Tail == FL.FreeListSize) {
        FL.Tail = 0;
        FL.TailPhaseBit = !FL.TailPhaseBit;
      }
    }
  }
}

uint64_t renamer::get_branch_mask() {
  // This function is used to get the branch mask for an instruction.
  // If all the bits are set in GBM then there are no available checkpoints for
  // the branch else the rightmost unset bit is set in order to obtain the
  // branch mask of a branch if (!(GBM & (GBM + 1))) { return GBM;
  //} else {
  // return GBM | (GBM + 1);
  //}
  // An instruction's initial branch mask is the value of the the GBM when the
  // instruction is renamed.
  return 0;
}

uint64_t renamer::rename_rsrc(uint64_t log_reg) {
  // This function is used to rename a single source register.
  // Inputs: log_reg: the logical register to rename
  // Return value: physical register name
  inc_usage_counter(RMT[log_reg].PhysicalRegisterMapping);
  return RMT[log_reg].PhysicalRegisterMapping;
}

uint64_t renamer::rename_rdst(uint64_t log_reg) {
  // This function is used to rename a single destination register.
  // Inputs: log_reg: the logical register to rename
  // Return value: physical register name
  assert(!(FL.Head == FL.Tail && FL.HeadPhaseBit == FL.TailPhaseBit));
  uint64_t DestinationPhysicalRegisterName = FL.FList[FL.Head];
  FL.Head++;
  if (FL.Head == FL.FreeListSize) {
    FL.Head = 0;
    FL.HeadPhaseBit = !FL.HeadPhaseBit;
  }
  PRF_ReadyBitArray[DestinationPhysicalRegisterName].ReadyBit = 0;
  unmap(RMT[log_reg].PhysicalRegisterMapping);
  RMT[log_reg].PhysicalRegisterMapping = DestinationPhysicalRegisterName;
  map(DestinationPhysicalRegisterName);
  inc_usage_counter(DestinationPhysicalRegisterName);
  return DestinationPhysicalRegisterName;
}

uint64_t renamer::PositionOfRightmostSetBit(uint64_t GBM) {
  return log2(GBM & -GBM) + 1;
}

uint64_t renamer::PositionOfRightmostUnsetBit(uint64_t GBM) {
  if (GBM == 0) {
    return 1;
  } else {
    return PositionOfRightmostSetBit(~GBM);
  }
}

void renamer::checkpoint() {
  // This function creates a new Checkpoint
  //  Inputs: none
  //  Output: none
  //  Tips:
  //  Allocating resources for a checkpoint
  //  *Assert that CheckpointBuffer has a free Checkpoint
  //  *Find a free checkpoint at the tail of the Checkpoint buffer
  //  Checkpoint the RMT
  //  Checkpoint the unmapped bits of the PRF
  //  Increment the usage counters of the registers present in the Checkpointed
  //  RMT Set all the counters associated with the checkpoint to 0
  // cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@checkpoint() called_Before"<<'\n';
  assert(!stall_checkpoint(1));
  // cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@checkpoint() called_after"<<'\n';
  for (uint64_t i = 0; i < RMTSize; i++) {
    CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail]
        .CheckpointedRMT[i]
        .RMT_Index = RMT[i].RMT_Index;
    CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail]
        .CheckpointedRMT[i]
        .PhysicalRegisterMapping = RMT[i].PhysicalRegisterMapping;
    inc_usage_counter(RMT[i].PhysicalRegisterMapping);
  }
  for (uint64_t j = 0; j < PRFSize; j++) {
    CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail]
        .CheckpointedUnmappedBits[j] = PRF[j].UnmappedBit;
  }
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail]
      .UncompletedInstructionCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail].LoadCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail].StoreCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail].BranchCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail].AMO_Flag = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail].CSR_Flag = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Tail].Exception_Flag = 0;
  // since we used the checkpoint at the tail of the Checkpoint buffer,
  // incrementing the tail so that it points to the next
  CheckpointBuffer.Tail++;
  if (CheckpointBuffer.Tail == CheckpointBuffer.CheckpointBufferSize) {
    CheckpointBuffer.Tail = 0;
    CheckpointBuffer.TailPhaseBit = !(CheckpointBuffer.TailPhaseBit);
  }
}

void renamer::free_checkpoint() {
  CheckpointBuffer.Head += 1;
  if (CheckpointBuffer.Head == CheckpointBuffer.CheckpointBufferSize) {
    CheckpointBuffer.Head = 0;
    CheckpointBuffer.HeadPhaseBit = !(CheckpointBuffer.HeadPhaseBit);
  }
}
// bool renamer::stall_dispatch(uint64_t bundle_inst) {
//     // The Dispatch Stage must stall if there are not enough free entries in
//     the Active List for all instructions in the current dispatch bundle.
// 	// Inputs: bundle_inst: number of instructions in current dispatch
// bundle
// 	// Return value: Return "true" (stall) if the Active List does not have
// enough space for all instructions in the dispatch bundle.

//     //Finding the number of free entries in the Active List to allocate
//     instructions uint64_t NumberOfFreeEntriesinActiveList; if (AL.Head ==
//     AL.Tail && AL.HeadPhaseBit == AL.TailPhaseBit) {
//         //The active list is empty and does not have any instructions
//         NumberOfFreeEntriesinActiveList = AL.ActiveListSize;
//     } else if (AL.Head == AL.Tail && AL.HeadPhaseBit != AL.TailPhaseBit) {
//         //The active list is full and there is no space for new instructions
//         NumberOfFreeEntriesinActiveList = 0;
//     } else if (AL.Head != AL.Tail && AL.HeadPhaseBit == AL.TailPhaseBit) {
//         NumberOfFreeEntriesinActiveList = AL.ActiveListSize - (AL.Tail -
//         AL.Head);
//     } else if (AL.Head != AL.Tail && AL.HeadPhaseBit != AL.TailPhaseBit) {
//         NumberOfFreeEntriesinActiveList = AL.Head - AL.Tail;
//     }

//     //If the number of free entries in the active list are greater than or
//     equal to the number of instructions in the current dispatch bundle then
//     return false else return true if (NumberOfFreeEntriesinActiveList >=
//     bundle_inst) {
//         return false;
//     } else {
//         return true;
//     }
// }

bool renamer::stall_checkpoint(uint64_t bundle_chkpts) {
  // The rename stage must stall if there are not enough free checkpoints in the
  // Checkpoint Buffer Inputs: bundle_chkpts: number of checkpoints that are
  // needed for the current rename bundle Return vaue: Return "true" (stall) if
  // the Checkpoint Buffer does not have enough free checkpoints that are
  // required by the rename bundle

  // Finding the number of free checkpoints in the Checkpoint Buffer
  uint64_t NumberOfAvailableCheckpointsInCheckpointBuffer =
      CheckpointBuffer.CheckpointBufferSize;
  if (CheckpointBuffer.Head == CheckpointBuffer.Tail &&
      CheckpointBuffer.HeadPhaseBit != CheckpointBuffer.TailPhaseBit)
    NumberOfAvailableCheckpointsInCheckpointBuffer =
        0; // There are no free checkpoints available in the checkpoint buffer

  else if (CheckpointBuffer.Head != CheckpointBuffer.Tail &&
           CheckpointBuffer.HeadPhaseBit != CheckpointBuffer.TailPhaseBit)
    NumberOfAvailableCheckpointsInCheckpointBuffer =
        CheckpointBuffer.Head - CheckpointBuffer.Tail;

  else if (CheckpointBuffer.Head != CheckpointBuffer.Tail &&
           CheckpointBuffer.HeadPhaseBit == CheckpointBuffer.TailPhaseBit)
    NumberOfAvailableCheckpointsInCheckpointBuffer =
        CheckpointBuffer.CheckpointBufferSize -
        (CheckpointBuffer.Tail - CheckpointBuffer.Head);

  // If the number of available checkpoints in the Checkpoint Buffer are greater
  // than or equal to bundle_chkpts then return false else return true
  // cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@Stall_checkpoint()\t"<<NumberOfAvailableCheckpointsInCheckpointBuffer<<'\t'<<bundle_chkpts
  // <<'\n';
  if (NumberOfAvailableCheckpointsInCheckpointBuffer >= bundle_chkpts)
    return false;
  else
    return true;
}

uint64_t renamer::get_checkpoint_ID(bool load, bool store, bool branch,
                                    bool amo, bool csr) {
  // This function returns the ID (i.e., checkpoint number, i.e., index into the
  // checkpoint buffer) of the nearest prior checkpoint. the nearest checkpoint
  // for the instruction woukd be Checkpoint buffer tail-1 based on the inputs
  // to the function, it increments the 4 counters and the 2 flags associated
  // with the checkpoint
  uint64_t Checkpoint_ID = 0;

  if (CheckpointBuffer.Tail > 0)
    Checkpoint_ID = CheckpointBuffer.Tail - 1;
  else
    Checkpoint_ID = CheckpointBuffer.CheckpointBufferSize - 1;

  if (load)
    CheckpointBuffer.Checkpoint[Checkpoint_ID].LoadCounter++;

  if (store)
    CheckpointBuffer.Checkpoint[Checkpoint_ID].StoreCounter++;

  if (branch)
    CheckpointBuffer.Checkpoint[Checkpoint_ID].BranchCounter++;

  if (amo)
    CheckpointBuffer.Checkpoint[Checkpoint_ID].AMO_Flag = true;

  if (csr)
    CheckpointBuffer.Checkpoint[Checkpoint_ID].CSR_Flag = true;

  CheckpointBuffer.Checkpoint[Checkpoint_ID].UncompletedInstructionCounter++;
  return Checkpoint_ID;
}

/*uint64_t renamer::dispatch_inst(bool dest_valid, uint64_t log_reg, uint64_t
phys_reg, bool load, bool store, bool branch, bool amo, bool csr, uint64_t PC)
{
    // This function dispatches a single instruction into the Active List.
    // Inputs: dest_valid: 1. If 'true', the instr. has a destination register,
otherwise it does not. If it does not, then the log_reg and phys_reg inputs
should be ignored.
    // 2. log_reg: Logical register number of the instruction's destination.
    // 3. phys_reg: Physical register number of the instruction's destination.
    // 4. load: If 'true', the instr. is a load, otherwise it isn't.
    // 5. store: If 'true', the instr. is a store, otherwise it isn't.
    // 6. branch: If 'true', the instr. is a branch, otherwise it isn't.
    // 7. amo: If 'true', this is an atomic memory operation.
    // 8. csr: If 'true', this is a system instruction.
    // 9. PC: Program counter of the instruction.
    // Return value: Return the instruction's index in the Active List.
    // Tips: Before dispatching the instruction into the Active List, assert
that the Active List isn't full: it is the user's responsibility to avoid a
structural hazard by calling stall_dispatch() in advance.
    assert(!stall_dispatch(1));
    AL.AL_entries[AL.Tail].DestinationFlag = dest_valid;
    if (dest_valid == true)
    {
        AL.AL_entries[AL.Tail].DestLogicalRegisterNumber = log_reg;
        AL.AL_entries[AL.Tail].DestPhysicalRegisterNumber = phys_reg;
        // PRF_ReadyBitArray[phys_reg].ReadyBit = 0;
    }
    AL.AL_entries[AL.Tail].LoadFlag = load;
    AL.AL_entries[AL.Tail].StoreFlag = store;
    AL.AL_entries[AL.Tail].BranchFlag = branch;
    AL.AL_entries[AL.Tail].amoFlag = amo;
    AL.AL_entries[AL.Tail].csrFlag = csr;
    AL.AL_entries[AL.Tail].PC = PC;
    AL.AL_entries[AL.Tail].ExceptionBit = 0;
    AL.AL_entries[AL.Tail].BranchMispredictionBit = 0;
    AL.AL_entries[AL.Tail].LoadViolationBit = 0;
    AL.AL_entries[AL.Tail].CompletedBit = 0;
    AL.AL_entries[AL.Tail].ValueMispredictionBit = 0;
    uint64_t x = AL.Tail;
    AL.Tail++;
    if (AL.Tail == AL.ActiveListSize)
    {
        AL.Tail = 0;
        AL.TailPhaseBit = !AL.TailPhaseBit;
    }
    // PRF_ReadyBitArray[phys_reg].ReadyBit = 0;
    return x;
}*/

bool renamer::is_ready(uint64_t phys_reg) {
  // Test the ready bit of the indicated physical register.
  // Returns 'true' if ready.
  if (PRF_ReadyBitArray[phys_reg].ReadyBit == 1) {
    return true;
  } else {
    return false;
  }
}

void renamer::clear_ready(uint64_t phys_reg) {
  // Clear the ready bit of the indicated physical register.
  PRF_ReadyBitArray[phys_reg].ReadyBit = 0;
}

uint64_t renamer::read(uint64_t phys_reg) {
  // Return the contents (value) of the indicated physical register.
  dec_usage_counter(phys_reg);
  return PRF[phys_reg].Value;
}

void renamer::set_ready(uint64_t phys_reg) {
  // Set the ready bit of the indicated physical register.
  PRF_ReadyBitArray[phys_reg].ReadyBit = 1;
}

void renamer::write(uint64_t phys_reg, uint64_t value) {
  // Write a value into the indicated physical register.
  dec_usage_counter(phys_reg);
  PRF[phys_reg].Value = value;
}

void renamer::set_complete(uint64_t Checkpoint_ID) {
  // Set the completed bit of the indicated entry in the Active List.
  CheckpointBuffer.Checkpoint[Checkpoint_ID].UncompletedInstructionCounter--;
}

/*void renamer::resolve(uint64_t AL_index, uint64_t branch_ID, bool correct)
{
    // This function is for handling branch resolution.
    // Inputs: 1. AL_index: Index of the branch in the Active List.
    // 2. branch_ID: This uniquely identifies the branch and the checkpoint in
question.  It was originally provided by the checkpoint function.
    // 3. correct: 'true' indicates the branch was correctly predicted, 'false'
indicates it was mispredicted and recovery is required.
    // Outputs: none.
    // Tips: While recovery is not needed in the case of a correct branch, some
actions are still required with respect to the GBM and all checkpointed GBMs:
    // * Remember to clear the branch's bit in the GBM.
    // * Remember to clear the branch's bit in all checkpointed GBMs.
    // In the case of a misprediction:
    // * Restore the GBM from the branch's checkpoint. Also make sure the
mispredicted branch's bit is cleared in the restored GBM, since it is now
resolved and its bit and checkpoint are freed.
    // * You don't have to worry about explicitly freeing the GBM bits and
checkpoints of branches that are after the mispredicted branch in program order.
The mere act of restoring the GBM from the checkpoint achieves this feat.
    // * Restore the RMT using the branch's checkpoint.
    // * Restore the Free List head pointer and its phase bit, using the
branch's checkpoint.
    // * Restore the Active List tail pointer and its phase bit corresponding to
the entry after the branch's entry.
    //   Hints: You can infer the restored tail pointer from the branch's
AL_index. You can infer the restored phase bit, using the phase bit of the
Active List head pointer, where the restored Active List tail pointer is with
respect to the Active List head pointer,
    //   and the knowledge that thw Active List can't be empty at this moment
(because the mispredicted branch is still in the Active List).
    // * Do NOT set the branch misprediction bit in the Active List. (Doing so
would cause a second, full squash when the branch reaches the head of the Active
List. We donÃ¢â‚¬â„¢t want or need that because we immediately recover within
this function.) if (correct == true)
    {
        // branch was correctly predicted and hence clearing the branch's bit in
the GBM and all the checkpointed GBMs GBM = GBM & (~((uint64_t)1 <<
(branch_ID)));
        // AL.AL_entries[AL_index].CompletedBit = 1;
        // int BranchCheckpointsSize = BranchCheckpoints.size();
        for (uint64_t j = 0; j < BranchCheckPointsSize; j++)
        {
            if ((BranchCheckpoints[j].CheckpointedGBM & ((uint64_t)1 <<
branch_ID)) != 0)
            {
                BranchCheckpoints[j].CheckpointedGBM =
BranchCheckpoints[j].CheckpointedGBM & (~((uint64_t)1 << (branch_ID)));
            }
        }
    }
    else if (correct == false)
    {
        // branch was incorrectly predicted/mispredicted
        // Restoring the GBM from branch's checkpoint
        // AL.AL_entries[AL_index].CompletedBit = 1;
        GBM = BranchCheckpoints[branch_ID].CheckpointedGBM;
        // Clearing the mispredicted branch's bit in the restred GBM as it is
resolved and now free GBM = GBM & (~((uint64_t)1 << (branch_ID)));
        // Restoring the RMT using branch's checkpoint
        // int RMTSize = BranchCheckpoints[branch_ID].CheckpointedRMT.size();
        for (uint64_t i = 0; i < RMTSize; i++)
        {
            RMT[i].RMT_Index =
BranchCheckpoints[branch_ID].CheckpointedRMT[i].RMT_Index;
            RMT[i].PhysicalRegisterMapping =
BranchCheckpoints[branch_ID].CheckpointedRMT[i].PhysicalRegisterMapping;
        }
        // Restore the Freelist head pointer and its phase bit using the
branch's checkpoint FL.Head =
BranchCheckpoints[branch_ID].CheckpointedFreeListHead; FL.HeadPhaseBit =
BranchCheckpoints[branch_ID].CheckpointedFreeListHeadPhaseBit;
        // Restore the active list tail pointer and its phase bit to the entry
after the branch's entry AL.Tail = AL_index + 1; if (AL.Tail ==
AL.ActiveListSize)
        {
            AL.Tail = 0;
        }
        if (AL.Head == AL.Tail)
        {
            // we know that the scenario of AL.Head == AL.Tail &&
AL.TailPhaseBit == AL.HeadPhaseBit does not exist because that condition would
mean that the active list is empty and we know that active list is not empty as
the branch instruction is still residing in the
            // active list
            AL.TailPhaseBit = !AL.HeadPhaseBit;
        }
        if (AL.Head != AL.Tail)
        {
            if (AL.Tail > AL.Head)
            {
                AL.TailPhaseBit = AL.HeadPhaseBit;
            }
            else if (AL.Tail < AL.Head)
            {
                AL.TailPhaseBit = !AL.HeadPhaseBit;
            }
        }
    }
}*/

uint64_t renamer::rollback(uint64_t chkpt_id, bool next, uint64_t &total_loads,
                           uint64_t &total_stores, uint64_t &total_branches) {
  //cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Rollback()called" << '\n';
  // cout << "********" << CheckpointBuffer.Head << "*******************" <<
  // chkpt_id << "**************" << CheckpointBuffer.Tail << "*************" <<
  // '\n';
  if (next)
    chkpt_id = (chkpt_id + 1) % (CheckpointBuffer.CheckpointBufferSize);
  // cout << "!!!!!!!" << CheckpointBuffer.Head << "!!!!!!!!!!!!!!!!" <<
  // chkpt_id << "!!!!!!!!!!!!!!!!" << CheckpointBuffer.Tail <<
  // "!!!!!!!!!!!!!!!!" << '\n';
  if (CheckpointBuffer.HeadPhaseBit == CheckpointBuffer.TailPhaseBit)
    assert((chkpt_id >= CheckpointBuffer.Head) &&
           (chkpt_id < CheckpointBuffer.Tail));
  else
    assert(!((chkpt_id >= CheckpointBuffer.Tail) &&
             (chkpt_id < CheckpointBuffer.Head)));

  for (uint64_t RMTSearch = 0; RMTSearch < RMTSize; RMTSearch++) {
    RMT[RMTSearch].RMT_Index = CheckpointBuffer.Checkpoint[chkpt_id]
                                   .CheckpointedRMT[RMTSearch]
                                   .RMT_Index;
    RMT[RMTSearch].PhysicalRegisterMapping =
        CheckpointBuffer.Checkpoint[chkpt_id]
            .CheckpointedRMT[RMTSearch]
            .PhysicalRegisterMapping;
  }
  for (uint64_t PRFSearch = 0; PRFSearch < PRFSize; PRFSearch++) {
    if (CheckpointBuffer.Checkpoint[chkpt_id]
            .CheckpointedUnmappedBits[PRFSearch] == false)
      map(PRF[PRFSearch].PhysicalRegisterNumber);
    else
      unmap(PRF[PRFSearch].PhysicalRegisterNumber);
  }

  uint64_t Index = chkpt_id;
  uint64_t SquashMask = 0;

  while (Index != CheckpointBuffer.Tail) {
    SquashMask += 1 << Index;
    Index = (Index + 1) % CheckpointBuffer.CheckpointBufferSize;
  }

  Index = (chkpt_id + 1) % CheckpointBuffer.CheckpointBufferSize;
  while (Index != CheckpointBuffer.Tail) {
    for (uint64_t RMTSearch = 0; RMTSearch < RMTSize; RMTSearch++) {
      uint64_t PRFUnderConsideration = CheckpointBuffer.Checkpoint[Index]
                                           .CheckpointedRMT[RMTSearch]
                                           .PhysicalRegisterMapping;
      assert(PRF[PRFUnderConsideration].UsageCounter > 0);
      dec_usage_counter(PRFUnderConsideration);
    }
    Index = (Index + 1) % CheckpointBuffer.CheckpointBufferSize;
  }
  CheckpointBuffer.Checkpoint[chkpt_id].UncompletedInstructionCounter = 0;
  CheckpointBuffer.Checkpoint[chkpt_id].LoadCounter = 0;
  CheckpointBuffer.Checkpoint[chkpt_id].StoreCounter = 0;
  CheckpointBuffer.Checkpoint[chkpt_id].BranchCounter = 0;
  CheckpointBuffer.Checkpoint[chkpt_id].AMO_Flag = false;
  CheckpointBuffer.Checkpoint[chkpt_id].CSR_Flag = false;
  CheckpointBuffer.Checkpoint[chkpt_id].Exception_Flag = false;

  total_loads = 0;
  total_stores = 0;
  total_branches = 0;
  Index = CheckpointBuffer.Head;
  while (Index != chkpt_id) {
    total_loads += CheckpointBuffer.Checkpoint[Index].LoadCounter;
    total_stores += CheckpointBuffer.Checkpoint[Index].StoreCounter;
    total_branches += CheckpointBuffer.Checkpoint[Index].BranchCounter;
    Index = (Index + 1) % CheckpointBuffer.CheckpointBufferSize;
  }
  CheckpointBuffer.Tail =
      (chkpt_id + 1) % CheckpointBuffer.CheckpointBufferSize;
  if (CheckpointBuffer.Head >= CheckpointBuffer.Tail)
    CheckpointBuffer.TailPhaseBit = !(CheckpointBuffer.HeadPhaseBit);
  else
    CheckpointBuffer.TailPhaseBit = CheckpointBuffer.HeadPhaseBit;
  return SquashMask;
}

/*bool renamer::precommit(bool &completed,bool &exception, bool &load_viol, bool
&br_misp, bool &val_misp,bool &load, bool &store, bool &branch, bool &amo, bool
&csr,uint64_t &PC) {
    // This function allows the caller to examine the instruction at the head of
the Active List.
    // Input arguments: none.
    // Return value:
    // * Return "true" if the Active List is NOT empty, i.e., there is an
instruction at the head of the Active List.
    // * Return "false" if the Active List is empty, i.e., there is no
instruction at the head of the Active List.
    // Output arguments: Simply return the following contents of the head entry
of the Active List.  These are don't-cares if the Active List is empty (you may
either return the contents of the head entry anyway, or not set these at all).
    // * completed bit
    // * exception bit
    // * load violation bit
    // * branch misprediction bit
    // * value misprediction bit
    // * load flag (indicates whether or not the instr. is a load)
    // * store flag (indicates whether or not the instr. is a store)
    // * branch flag (indicates whether or not the instr. is a branch)
    // * amo flag (whether or not instr. is an atomic memory operation)
    // * csr flag (whether or not instr. is a system instruction)
    // * program counter of the instruction
    completed = AL.AL_entries[AL.Head].CompletedBit;
    exception = AL.AL_entries[AL.Head].ExceptionBit;
    load_viol = AL.AL_entries[AL.Head].LoadViolationBit;
    br_misp = AL.AL_entries[AL.Head].BranchMispredictionBit;
    val_misp = AL.AL_entries[AL.Head].ValueMispredictionBit;
    load = AL.AL_entries[AL.Head].LoadFlag;
    store = AL.AL_entries[AL.Head].StoreFlag;
    branch = AL.AL_entries[AL.Head].BranchFlag;
    amo = AL.AL_entries[AL.Head].amoFlag;
    csr = AL.AL_entries[AL.Head].csrFlag;
    PC = AL.AL_entries[AL.Head].PC;
    if (AL.Head == AL.Tail && AL.HeadPhaseBit == AL.TailPhaseBit) {
        //The active list is empty and does not have any instructions
        return false;
    } else {
        //The active list is not empty and there is an instruction at the head
of the active list return true;
   }
}*/

bool renamer::precommit(uint64_t &chkpt_id, uint64_t &num_loads,
                        uint64_t &num_stores, uint64_t &num_branches, bool &amo,
                        bool &csr, bool &exception) {
  uint64_t CheckPointUsageCounter = 0;
  if (CheckpointBuffer.HeadPhaseBit == CheckpointBuffer.TailPhaseBit)
    CheckPointUsageCounter = CheckpointBuffer.Tail - CheckpointBuffer.Head;
  else
    CheckPointUsageCounter = CheckpointBuffer.CheckpointBufferSize -
                             (CheckpointBuffer.Head - CheckpointBuffer.Tail);
  if (((CheckPointUsageCounter > 1) ||
       CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].Exception_Flag) &&
      (CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
           .UncompletedInstructionCounter == 0)) {
    chkpt_id = CheckpointBuffer.Head;
    num_loads = CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].LoadCounter;
    num_stores =
        CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].StoreCounter;
    num_branches =
        CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].BranchCounter;
    amo = CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].AMO_Flag;
    csr = CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].CSR_Flag;
    exception =
        CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].Exception_Flag;
    return true;
  } else {
    return false;
  }
}
/*void renamer::commit()
{
    // This function commits the instruction at the head of the Active List.
    // Tip (optional but helps catch bugs): Before committing the head
instruction, assert that it is valid to do so (use assert() from standard
library). Specifically, assert that all of the following are true:
    // - there is a head instruction (the active list isn't empty)
    // - the head instruction is completed
    // - the head instruction is not marked as an exception
    // - the head instruction is not marked as a load violation
    // It is the caller's (pipeline's) duty to ensure that it is valid to commit
the head instruction BEFORE calling this function (by examining the flags
returned by "precommit()" above). This is why you should assert() that it is
valid to commit the
    // head instruction and otherwise cause the simulator to exit.

    // Asserting that the active list is not empty so there is an instruction at
the head of the active list assert(!(AL.Head == AL.Tail && AL.HeadPhaseBit ==
AL.TailPhaseBit));
    // Asserting that the head instruction is completed
    assert(AL.AL_entries[AL.Head].CompletedBit == true);
    // Asserting that the head instruction is not marked as an exception
    assert(AL.AL_entries[AL.Head].ExceptionBit != true);
    // Asserting that the head instruction is not marked as a load violation
    assert(AL.AL_entries[AL.Head].LoadViolationBit != true);
    // Freeing the prior committed logical register from the AMT i.e., add the
register into the free list and then updating the AMT with the current committed
physical register if (AL.AL_entries[AL.Head].DestinationFlag == true)
    {
        FL.FList[FL.Tail] =
AMT[AL.AL_entries[AL.Head].DestLogicalRegisterNumber].PhysicalRegisterMapping;
        FL.Tail++;
        if (FL.Tail == FL.FreeListSize)
        {
            FL.Tail = 0;
            FL.TailPhaseBit = !FL.TailPhaseBit;
        }
        AMT[AL.AL_entries[AL.Head].DestLogicalRegisterNumber].PhysicalRegisterMapping
= AL.AL_entries[AL.Head].DestPhysicalRegisterNumber;
    }
    AL.Head++;
    if (AL.Head == AL.ActiveListSize)
    {
        AL.Head = 0;
        AL.HeadPhaseBit = !AL.HeadPhaseBit;
    }
}*/

void renamer::commit(uint64_t log_reg) {
  // This function commits the instruction at the head of the Active List.
  // Tip (optional but helps catch bugs): Before committing the head
  // instruction, assert that it is valid to do so (use assert() from standard
  // library). Specifically, assert that all of the following are true:
  // - there is a head instruction (the active list isn't empty)
  // - the head instruction is completed
  // - the head instruction is not marked as an exception
  // - the head instruction is not marked as a load violation
  // It is the caller's (pipeline's) duty to ensure that it is valid to commit
  // the head instruction BEFORE calling this function (by examining the flags
  // returned by "precommit()" above). This is why you should assert() that it
  // is valid to commit the head instruction and otherwise cause the simulator
  // to exit.

  uint64_t CheckPointUsageCounter = 0;
  if (CheckpointBuffer.HeadPhaseBit == CheckpointBuffer.TailPhaseBit)
    CheckPointUsageCounter = CheckpointBuffer.Tail - CheckpointBuffer.Head;
  else
    CheckPointUsageCounter = CheckpointBuffer.CheckpointBufferSize -
                             (CheckpointBuffer.Head - CheckpointBuffer.Tail);

  assert(CheckPointUsageCounter > 1);
  assert(CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
             .UncompletedInstructionCounter == 0);
  assert(CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
             .CheckpointedRMT[log_reg]
             .RMT_Index == log_reg);

  uint64_t PRFUnderConsideration =
      CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
          .CheckpointedRMT[log_reg]
          .PhysicalRegisterMapping;

  assert(PRF[PRFUnderConsideration].UsageCounter > 0);

  PRF[PRFUnderConsideration].UnmappedBit = 1;
  for (uint64_t i = 0; i < RMTSize; i++) {
    if (RMT[i].PhysicalRegisterMapping == PRFUnderConsideration) {
      PRF[PRFUnderConsideration].UnmappedBit = 0;
      break;
    }
  }
  dec_usage_counter(PRFUnderConsideration);
}

/*void renamer::squash()
{
    // Squash the renamer class.
    // Squash all instructions in the Active List and think about which
sructures in your renamer class need to be restored, and how.
    // After this function is called, the renamer should be rolled-back to the
committed state of the machine and all renamer state should be consistent with
an empty pipeline.

    // Squashing all the instructions in the active list i.e., making the active
list empty AL.Tail = AL.Head; AL.TailPhaseBit = AL.HeadPhaseBit;
    // Restoring the Free List to its initial state i.e., making the free list
full FL.Head = FL.Tail; FL.HeadPhaseBit = !FL.TailPhaseBit;
    // In order to rollback the renamer to the committed state of the machine,
the AMT should be copied to RMT indicating the committed states as consistent
with an empty pipeline GBM = 0;
    // int AMTSize = AMT.size();
    for (uint64_t i = 0; i < AMTSize; i++)
    {
        RMT[i].RMT_Index = AMT[i].AMT_Index;
        RMT[i].PhysicalRegisterMapping = AMT[i].PhysicalRegisterMapping;
    }
    // int PRF_ReadyBitArraySize = PRF_ReadyBitArray.size();
    for (uint64_t j = 0; j < PRFSize; j++)
    {
        PRF_ReadyBitArray[j].ReadyBit = 0;
    }

    for (uint64_t k = 0; k < AMTSize; k++)
    {
        PRF_ReadyBitArray[AMT[k].PhysicalRegisterMapping].ReadyBit = 1;
    }
}*/

void renamer::squash() {
  // Squash the renamer class.
  // Squash all instructions in the Active List and think about which sructures
  // in your renamer class need to be restored, and how. After this function is
  // called, the renamer should be rolled-back to the committed state of the
  // machine and all renamer state should be consistent with an empty pipeline.

  // Squashing all the instructions in the active list i.e., making the active
  // list empty
  // cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Squash()called" << '\n';

  for (uint64_t PRFSearch = 0; PRFSearch < PRFSize; PRFSearch++) {
    // PRF_ReadyBitArray.push_back({PRFSearch, 0});
    if (CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
            .CheckpointedUnmappedBits[PRFSearch] == false)
      map(PRF[PRFSearch].PhysicalRegisterNumber);
    else
      unmap(PRF[PRFSearch].PhysicalRegisterNumber);
  }

  for (uint64_t RMTSearch = 0; RMTSearch < RMTSize; RMTSearch++) {
    RMT[RMTSearch].RMT_Index =
        CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
            .CheckpointedRMT[RMTSearch]
            .RMT_Index;
    RMT[RMTSearch].PhysicalRegisterMapping =
        CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
            .CheckpointedRMT[RMTSearch]
            .PhysicalRegisterMapping;
    // PRF_ReadyBitArray.push_back({RMT[RMTSearch].PhysicalRegisterMapping, 1});
  }

  uint64_t CheckPointUsageCounter = 0;
  if (CheckpointBuffer.HeadPhaseBit == CheckpointBuffer.TailPhaseBit)
    CheckPointUsageCounter = CheckpointBuffer.Tail - CheckpointBuffer.Head;
  else
    CheckPointUsageCounter = CheckpointBuffer.CheckpointBufferSize -
                             (CheckpointBuffer.Head - CheckpointBuffer.Tail);

  for (uint64_t CheckPointSearch = 1;
       CheckPointSearch < (CheckPointUsageCounter); CheckPointSearch++) {
    uint64_t Index = (CheckpointBuffer.Head + CheckPointSearch) %
                     CheckpointBuffer.CheckpointBufferSize;
    for (uint64_t RMTSearch = 0; RMTSearch < RMTSize; RMTSearch++) {
      uint64_t PRFUnderConsideration = CheckpointBuffer.Checkpoint[Index]
                                           .CheckpointedRMT[RMTSearch]
                                           .PhysicalRegisterMapping;
      dec_usage_counter(PRFUnderConsideration);
    }
  }

  CheckpointBuffer.Checkpoint[CheckpointBuffer.Head]
      .UncompletedInstructionCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].LoadCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].StoreCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].BranchCounter = 0;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].AMO_Flag = false;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].CSR_Flag = false;
  CheckpointBuffer.Checkpoint[CheckpointBuffer.Head].Exception_Flag = false;
  CheckpointBuffer.TailPhaseBit = CheckpointBuffer.HeadPhaseBit;
  CheckpointBuffer.Tail = CheckpointBuffer.Head + 1;
  if (CheckpointBuffer.Tail == CheckpointBuffer.CheckpointBufferSize) {
    CheckpointBuffer.Tail = 0;
    CheckpointBuffer.TailPhaseBit = !(CheckpointBuffer.TailPhaseBit);
  }
}

void renamer::set_exception(uint64_t Checkpoint_ID) {
  // Function for individually setting the exception flag of a checkpoint
  CheckpointBuffer.Checkpoint[Checkpoint_ID].Exception_Flag = true;
}

void renamer::set_load_violation(uint64_t AL_index) {
  // Function for individually setting the load violation bit of the indicated
  // entry in the Active List.
  //*@*AL.AL_entries[AL_index].LoadViolationBit = true;
}

void renamer::set_branch_misprediction(uint64_t AL_index) {
  // Function for individually setting the branch misprediction bit of the
  // indicated entry in the Active List.
  //*@*AL.AL_entries[AL_index].BranchMispredictionBit = true;
}

void renamer::set_value_misprediction(uint64_t AL_index) {
  // Function for individually setting the value misprediction bit of the
  // indicated entry in the Active List.
  //*@*AL.AL_entries[AL_index].ValueMispredictionBit = true;
}

bool renamer::get_exception(uint64_t Checkpoint_ID) {
  // Query the exception bit of the indicated entry in the Active List.
  return CheckpointBuffer.Checkpoint[Checkpoint_ID].Exception_Flag;
}