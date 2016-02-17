#ifndef DATATYPES
#define DATATYPES

struct position
{
    float x,y,z;
};

struct pixel
{
    unsigned short row;
    unsigned short col;
};

struct Joint
{
    position joint_position;
    float    joint_positions_confidence;
};



#endif // DATATYPES

