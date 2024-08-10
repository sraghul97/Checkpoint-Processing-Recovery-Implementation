#include "mmu.h"
#include "pipeline.h"
#include "trap.h"

void pipeline_t::retire(size_t &instret, size_t instret_limit)
{
    // bool head_valid;
    // bool completed, exception, load_viol, br_misp, val_misp, load, store, branch, amo, csr;
    reg_t offending_PC;
    bool load, store;
    bool amo_success;
    trap_t *trap = NULL; // Supress uninitialized warning.

    // FIX_ME #17a
    // Call the precommit() function of the renamer module.  This tells the
    // renamer module to return the state of the instruction at the head of its
    // Active List, if any.  The renamer does not take any action, itself.
    // Moreover:
    // * The precommit() function returns 'true' if there is a head instruction
    // (active list not empty)
    //   and 'false' otherwise (active list empty).
    // * The precommit() function also modifies arguments that THIS function must
    // examine and act upon.
    //
    // Tips:
    // 1. Call the renamer module's precommit() function with the following
    // arguments in this order:
    //    'completed', 'exception', 'load_viol', 'br_misp', 'val_misp', 'load',
    //    'store', 'branch', 'amo', 'csr', 'offending_PC'. These are local
    //    variables, already declared above.
    // 2. Use the return value of the precommit() function call, to set the
    // already-declared local
    //    variable 'head_valid'.
    // 3. Study the code that follows the code that you added, and simply note the
    // following observations:
    //    * The retire stage only does something if the renamer module's
    //    precommit() function signals
    //      a non-empty active list and that the head instruction is complete: "if
    //      (head_valid && completed)".
    //    * If the completed head instruction is not an exception -- "if
    //    (!exception)" -- some additional
    //      processing is needed for atomic memory operations (execute_amo()) and
    //      system instructions (execute_csr()). Note that an amo or csr
    //      instruction may set the exception flag at this point due to deferring
    //      their final execution to the retire stage.
    //    * If, even after the additional processing of the above-deferred
    //    instructions, the completed head instruction
    //      is not an exception or load violation -- "if (!exception &&
    //      !load_viol)" -- it is committed. Loads and stores are committed in the
    //      LSU.  Branches are finalized in the branch prediction unit. Committed
    //      results are checked against the functional simulator via the checker()
    //      function call. Serializing instructions, such as atomics and system
    //      instructions, resume the stalled fetch unit. If the simulator ever
    //      sets the br_misp and val_misp flags, specifically due to implementing
    //      "approach #1 recovery" for branch and value mispredictions, then these
    //      trigger a complete pipeline squash.
    //    * Alternatively, if the completed head instruction is not an exception
    //    but a load violation --
    //      "else if (!exception && load_viol)" -- all instructions including the
    //      load instruction are squashed.
    //    * Alternatively, if the completed head instruction is an exception, the
    //    trap is taken and the pipeline
    //      is squashed including the offending instruction.

    // FIX_ME #17a BEGIN
    if (RETSTATE.state == RETIRE_IDLE)
    {
        bool proceed = REN->precommit(RETSTATE.chkpt_id, RETSTATE.num_loads_left, RETSTATE.num_stores_left, RETSTATE.num_branches_left, RETSTATE.amo, RETSTATE.csr, RETSTATE.exception);

        if (!proceed)
            return;

        // Sanity checks of the 'amo' and 'csr' flags.
        assert(!RETSTATE.amo || IS_AMO(PAY.buf[PAY.head].flags));
        assert(!RETSTATE.csr || IS_CSR(PAY.buf[PAY.head].flags));
        if (RETSTATE.amo || RETSTATE.csr)
        {
            // There should be only 1 instruction – the amo or csr –
            // between the oldest and next oldest checkpoint.
            // So the following assertions should succeed.
            assert(RETSTATE.num_loads_left <= 1);
            assert(RETSTATE.num_stores_left <= 1);
            assert(RETSTATE.num_branches_left == 0);
            // load and store are declared as local variables (bool)
            load = (RETSTATE.num_loads_left > 0);
            store = (RETSTATE.num_stores_left > 0);
        }
        if (!RETSTATE.exception)
        {
            if (RETSTATE.amo && !(load || store))
            { // amo, excluding load-with-reservation (LR) and store-conditional (SC)
                RETSTATE.exception = execute_amo();
            }
            else if (RETSTATE.csr)
            {
                RETSTATE.exception = execute_csr();
            }
            // This is probably optional.
            // Just doing it out of completeness and adapting existing code.
            if (RETSTATE.exception)
                REN->set_exception(RETSTATE.chkpt_id);
        }
        if (RETSTATE.exception)
        {
            //cout<<"###################################retire1_exception()called "<<RETSTATE.exception<<"#############################################"<<'\n';
            trap = PAY.buf[PAY.head].trap.get();
            offending_PC = PAY.buf[PAY.head].pc; ////////////////////////////////////////////////////////////
            // CSR exceptions are micro-architectural exceptions and are
            // not defined by the ISA. These must be handled exclusively by
            // the micro-arch and is different from other exceptions specified
            // in the ISA.
            // This is a serialize trap - Refetch the CSR instruction
            reg_t jump_PC;
            if (trap->cause() == CAUSE_CSR_INSTRUCTION)
            {
                jump_PC = offending_PC;
            }
            else
            {
                jump_PC = take_trap(*trap, offending_PC);
            }

            // Keep track of the number of retired instructions.
            instret++;
            num_insn++;
            inc_counter(commit_count);
            inc_counter(exception_count);

            // Compare pipeline simulator against functional simulator.
            checker();

            // Squash the pipeline.
            squash_complete(jump_PC);
            inc_counter(recovery_count);

            // Flush PAY.
            PAY.clear();

            update_timer(&state, 1); // Update timer by 1 retired instr.///******Patch
            assert(instret <= instret_limit);///******Patch

            return;
        }

        else
        {
            RETSTATE.state = RETIRE_BULK_COMMIT;
            RETSTATE.log_reg = 0;
			return;
        }
    }
    else if (RETSTATE.state == RETIRE_BULK_COMMIT)
    {
        for (uint64_t Counter = 0; Counter < RETIRE_WIDTH; Counter++)
        {
            if (RETSTATE.num_loads_left > 0)
            {
                LSU.train(true);
                amo_success = LSU.commit(true, RETSTATE.amo);
                assert(amo_success);
                RETSTATE.num_loads_left -= 1;
            }
            if (RETSTATE.num_stores_left > 0)
            {
                LSU.train(false);
                amo_success = LSU.commit(false, RETSTATE.amo);
                assert(amo_success);
                RETSTATE.num_stores_left -= 1;
            }
            if (RETSTATE.num_branches_left > 0)
            {
                FetchUnit->commit();
                RETSTATE.num_branches_left -= 1;
            }
            if (RETSTATE.log_reg < NXPR + NFPR)
            {
                REN->commit(RETSTATE.log_reg);
                RETSTATE.log_reg += 1;
            }
            if ((RETSTATE.num_loads_left == 0) && (RETSTATE.num_stores_left == 0) && (RETSTATE.num_branches_left == 0) && (RETSTATE.log_reg == NXPR + NFPR))
            {
                REN->free_checkpoint();
                RETSTATE.state = RETIRE_FINALIZE;
                return;
            }
        }
		return;
    }
    else if (RETSTATE.state == RETIRE_FINALIZE)
    {

        while ((PAY.buf[PAY.head].Checkpoint_ID == RETSTATE.chkpt_id) && (PAY.head != PAY.tail))
        {
            if (IS_FP_OP(PAY.buf[PAY.head].flags))
            {
                // post the FP exception bit to CSR fflags (the Accrued Exception Flags)
                get_state()->fflags |= PAY.buf[PAY.head].fflags;
            }

            // Check results.
            checker();

            // Keep track of the number of retired instructions.
            num_insn++;
            instret++;
            inc_counter(commit_count);
            if (PAY.buf[PAY.head].split && PAY.buf[PAY.head].upper)
                num_insn_split++;

            // Sanity checks of the 'amo' and 'csr' flags.
            assert(!RETSTATE.amo || IS_AMO(PAY.buf[PAY.head].flags));
            assert(!RETSTATE.csr || IS_CSR(PAY.buf[PAY.head].flags));
            if (RETSTATE.amo || RETSTATE.csr)
            { // Resume the stalled fetch unit after committing a
              // serializing instruction.

                insn_t inst = PAY.buf[PAY.head].inst;
                reg_t next_inst_pc;
                if ((inst.funct3() == FN3_SC_SB) &&
                    (inst.funct12() == FN12_SRET)) // SRET instruction.
                    next_inst_pc = state.epc;
                else
                    next_inst_pc = INCREMENT_PC(PAY.buf[PAY.head].pc);

                // The serializing instruction stalled the fetch unit so the pipeline is
                // now empty. Resume fetch.
                FetchUnit->flush(next_inst_pc);
            }

            // Pop the instruction from PAY.
            if (!PAY.buf[PAY.head].split)
                PAY.pop();
            PAY.pop();

            update_timer(&state, 1); // Update timer by 1 retired instr.
            // Pause, but remain in the RETIRE_FINALIZE state for
            // the next cycle, if it’s time for an HTIF tick,
            // as this will change state.
            if (instret == instret_limit)
                return; // Pause and remain in the state RETIRE_FINALIZE.
        }
        RETSTATE.state = RETIRE_IDLE;
		return;
    }
}

bool pipeline_t::execute_amo()
{
    unsigned int index = PAY.head;
    insn_t inst = PAY.buf[index].inst;
    reg_t read_amo_value = 0xdeadbeef;
    bool exception = false;

    try
    {
        if (inst.funct3() == FN3_AMO_W)
        {
            read_amo_value = mmu->load_int32(PAY.buf[index].A_value.dw);
            uint32_t write_amo_value;
            switch (inst.funct5())
            {
            case FN5_AMO_SWAP:
                write_amo_value = PAY.buf[index].B_value.dw;
                break;
            case FN5_AMO_ADD:
                write_amo_value = PAY.buf[index].B_value.dw + read_amo_value;
                break;
            case FN5_AMO_XOR:
                write_amo_value = PAY.buf[index].B_value.dw ^ read_amo_value;
                break;
            case FN5_AMO_AND:
                write_amo_value = PAY.buf[index].B_value.dw & read_amo_value;
                break;
            case FN5_AMO_OR:
                write_amo_value = PAY.buf[index].B_value.dw | read_amo_value;
                break;
            case FN5_AMO_MIN:
                write_amo_value = std::min(int32_t(PAY.buf[index].B_value.dw),
                                           int32_t(read_amo_value));
                break;
            case FN5_AMO_MAX:
                write_amo_value = std::max(int32_t(PAY.buf[index].B_value.dw),
                                           int32_t(read_amo_value));
                break;
            case FN5_AMO_MINU:
                write_amo_value = std::min(uint32_t(PAY.buf[index].B_value.dw),
                                           uint32_t(read_amo_value));
                break;
            case FN5_AMO_MAXU:
                write_amo_value = std::max(uint32_t(PAY.buf[index].B_value.dw),
                                           uint32_t(read_amo_value));
                break;
            default:
                assert(0);
                break;
            }
            mmu->store_uint32(PAY.buf[index].A_value.dw, write_amo_value);
        }
        else if (inst.funct3() == FN3_AMO_D)
        {
            read_amo_value = mmu->load_int64(PAY.buf[index].A_value.dw);
            reg_t write_amo_value;
            switch (inst.funct5())
            {
            case FN5_AMO_SWAP:
                write_amo_value = PAY.buf[index].B_value.dw;
                break;
            case FN5_AMO_ADD:
                write_amo_value = PAY.buf[index].B_value.dw + read_amo_value;
                break;
            case FN5_AMO_XOR:
                write_amo_value = PAY.buf[index].B_value.dw ^ read_amo_value;
                break;
            case FN5_AMO_AND:
                write_amo_value = PAY.buf[index].B_value.dw & read_amo_value;
                break;
            case FN5_AMO_OR:
                write_amo_value = PAY.buf[index].B_value.dw | read_amo_value;
                break;
            case FN5_AMO_MIN:
                write_amo_value = std::min(int64_t(PAY.buf[index].B_value.dw),
                                           int64_t(read_amo_value));
                break;
            case FN5_AMO_MAX:
                write_amo_value = std::max(int64_t(PAY.buf[index].B_value.dw),
                                           int64_t(read_amo_value));
                break;
            case FN5_AMO_MINU:
                write_amo_value = std::min(PAY.buf[index].B_value.dw, read_amo_value);
                break;
            case FN5_AMO_MAXU:
                write_amo_value = std::max(PAY.buf[index].B_value.dw, read_amo_value);
                break;
            default:
                assert(0);
                break;
            }
            mmu->store_uint64(PAY.buf[index].A_value.dw, write_amo_value);
        }
        else
        {
            assert(0);
        }
    }
    catch (mem_trap_t &t)
    {
        exception = true;
        assert(t.cause() == CAUSE_FAULT_STORE ||
               t.cause() == CAUSE_MISALIGNED_STORE);
        PAY.buf[index].trap.post(t);
    }

    // Record the loaded value in the payload buffer for checking purposes.
    PAY.buf[index].C_value.dw = read_amo_value;

    // Write the loaded value to the destination physical register.
    // "amoswap" may have rd=x0 (effectively no destination register) to implement
    // a "sequentially consistent store" (see RISCV ISA spec).
    // assert(PAY.buf[index].C_valid);
    if (PAY.buf[index].C_valid)
    {
        REN->set_ready(PAY.buf[index].C_phys_reg);
        REN->write(PAY.buf[index].C_phys_reg, PAY.buf[index].C_value.dw);
    }

    return (exception);
}

bool pipeline_t::execute_csr()
{
    unsigned int index = PAY.head;
    insn_t inst = PAY.buf[index].inst;
    pipeline_t *p =
        this; // *p is assumed by the validate_csr and require_supervisor macros.
    int csr;
    bool exception = false;

    // CSR instructions:
    // 1. read the addressed CSR and write its old value into a destination
    // register,
    // 2. modify the old value to get a new value, and
    // 3. write the new value into the addressed CSR.
    reg_t old_value;
    reg_t new_value;

    try
    {
        if (inst.funct3() != FN3_SC_SB)
        {
            switch (inst.funct3())
            {
            case FN3_CLR:
                csr = validate_csr(PAY.buf[index].CSR_addr, true);
                old_value = get_pcr(csr);
                new_value = (old_value & ~PAY.buf[index].A_value.dw);
                set_pcr(csr, new_value);
                break;
            case FN3_RW:
                csr = validate_csr(PAY.buf[index].CSR_addr, true);
                old_value = get_pcr(csr);
                new_value = PAY.buf[index].A_value.dw;
                set_pcr(csr, new_value);
                break;
            case FN3_SET:
                csr = validate_csr(PAY.buf[index].CSR_addr,
                                   (PAY.buf[index].A_log_reg != 0));
                old_value = get_pcr(csr);
                new_value = (old_value | PAY.buf[index].A_value.dw);
                set_pcr(csr, new_value);
                break;
            case FN3_CLR_IMM:
                csr = validate_csr(PAY.buf[index].CSR_addr, true);
                old_value = get_pcr(csr);
                new_value = (old_value & ~(reg_t)PAY.buf[index].A_log_reg);
                set_pcr(csr, new_value);
                break;
            case FN3_RW_IMM:
                csr = validate_csr(PAY.buf[index].CSR_addr, true);
                old_value = get_pcr(csr);
                new_value = (reg_t)PAY.buf[index].A_log_reg;
                set_pcr(csr, new_value);
                break;
            case FN3_SET_IMM:
                csr = validate_csr(PAY.buf[index].CSR_addr, true);
                old_value = get_pcr(csr);
                new_value = (old_value | (reg_t)PAY.buf[index].A_log_reg);
                set_pcr(csr, new_value);
                break;
            default:
                assert(0);
                break;
            }
        }
        else if (inst.funct12() == FN12_SRET)
        {
            // This is a macro defined in decode.h.
            // This will throw a privileged_instruction trap if processor not in
            // supervisor mode.
            require_supervisor;
            csr = validate_csr(PAY.buf[index].CSR_addr, true);
            old_value = get_pcr(csr);
            new_value =
                ((old_value & ~(SR_S | SR_EI)) | ((old_value & SR_PS) ? SR_S : 0) |
                 ((old_value & SR_PEI) ? SR_EI : 0));
            set_pcr(csr, new_value);
        }
        else
        {
            // SCALL and SBREAK.
            // These skip the IQ and execution lanes (completed in Dispatch Stage).
            assert(0);
        }

        if (PAY.buf[index].C_valid)
        {
            // Write the result (old value of CSR) to the payload buffer for checking
            // purposes.
            PAY.buf[index].C_value.dw = old_value;
            // Write the result (old value of CSR) to the physical destination
            // register.
            REN->set_ready(PAY.buf[index].C_phys_reg);
            REN->write(PAY.buf[index].C_phys_reg, PAY.buf[index].C_value.dw);
        }
    }
    catch (trap_t &t)
    {
        exception = true;
        assert(t.cause() == CAUSE_PRIVILEGED_INSTRUCTION ||
               t.cause() == CAUSE_FP_DISABLED);
        PAY.buf[index].trap.post(t);
    }
    catch (serialize_t &s)
    {
        exception = true;
        PAY.buf[index].trap.post(trap_csr_instruction());
    }

    return (exception);
}
