#ifndef PIPELINE_REGISTER_H
#define PIPELINE_REGISTER_H

class pipeline_register
{

public:
    bool valid;                     // valid instruction
    unsigned int index;             // index into instruction payload buffer
    //unsigned long long branch_mask; // branches that this instruction depends on
    unsigned long long Checkpoint_ID; // Checkpoint that this instruction should move on to

    pipeline_register(); // constructor
};

#endif // PIPELINE_REGISTER_H
