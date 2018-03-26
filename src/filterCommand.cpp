/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/NetInt32.h>
#include <yarp/os/Vocab.h>

//memset
#include <memory.h> 

#include "headers/filterCommand.h"

YARP_BEGIN_PACK
class FilterCommandPortContentHeader
{
public:
    yarp::os::NetInt32 n_points;
    yarp::os::NetInt32 n_inputs;

    FilterCommandPortContentHeader() : n_points(0), n_inputs(0) {}
};
YARP_END_PACK

yarp::sig::FilterCommand::FilterCommand() :
    n_points_alloc(0), n_points(0), points_storage(0),
    n_inputs_alloc(0), n_inputs(0), inputs_storage(0) {}
    
yarp::sig::FilterCommand::FilterCommand(const FilterCommand &f) : 
    points_storage(0), inputs_storage(0)
{
    n_points_alloc = n_points = f.n_points;
    n_inputs_alloc = n_inputs = f.n_inputs;

    tag_value = f.tag_value;
    cmd_value = f.cmd_value;

    if (f.points_storage != 0)
    {
        points_storage = new double[3 * n_points_alloc];
	memcpy(points_storage, f.points_storage, 3 * n_points * sizeof(double));
    }

    if (f.inputs_storage != 0)
    {
        inputs_storage = new double[3 * n_inputs_alloc];
	memcpy(inputs_storage, f.inputs_storage, 3 * n_inputs * sizeof(double));
    }
}

yarp::sig::FilterCommand& yarp::sig::FilterCommand::operator=(const FilterCommand& f)
{
    // quick implementation
    // there is room for improvements
    if (this == &f)
	return *this;

    tag_value = f.tag_value;
    cmd_value = f.cmd_value;
    
    if (f.points_storage != 0)
    {
	if (n_points_alloc < f.n_points)
	{
	    delete [] points_storage;
	    points_storage = new double[3 * f.n_points];
	    n_points_alloc = f.n_points;
	}
	memcpy(points_storage, f.points_storage, 3 * f.n_points * sizeof(double));
	n_points = f.n_points;
    }
    else
    {
	if (points_storage)
	    delete [] points_storage;
	n_points_alloc = n_points = 0;
	points_storage = 0;
    }

    if (f.inputs_storage != 0)
    {
	if (n_inputs_alloc < f.n_inputs)
	{
	    delete [] inputs_storage;
	    inputs_storage = new double[3 * f.n_inputs];
	    n_inputs_alloc = f.n_inputs;
	}
	memcpy(inputs_storage, f.inputs_storage, 3 * f.n_inputs * sizeof(double));
	n_inputs = f.n_inputs;
    }
    else
    {
	if (inputs_storage)
	    delete [] inputs_storage;
	n_inputs_alloc = n_inputs = 0;
	inputs_storage = 0;
    }
}

yarp::sig::FilterCommand::~FilterCommand()
{
    if (points_storage != 0)
    	delete [] points_storage;

    if (inputs_storage != 0)
    	delete [] inputs_storage;
}

bool yarp::sig::FilterCommand::addPoint(const yarp::sig::Vector& point)
{
    if (point.size() != 3)
	return false;

    if (n_points == n_points_alloc)
    {
	double* tmp = new double[3 * (n_points_alloc + 5)];
	if (points_storage)
	    memcpy(tmp, points_storage, 3 * n_points_alloc * sizeof(double));
	delete [] points_storage;
	points_storage = tmp;
	n_points_alloc += 5;
    }
    
    for (size_t i=0; i<3; i++)
	points_storage[n_points * 3 + i] = point[i];

    n_points++;

    return true;
}

bool yarp::sig::FilterCommand::addInput(const yarp::sig::Vector& input)
{
    if (input.size() != 3)
	return false;

    if (n_inputs == n_inputs_alloc)
    {
	double* tmp = new double[3 * (n_inputs_alloc + 5)];
	if (inputs_storage)
	    memcpy(tmp, inputs_storage, 3 * n_inputs_alloc * sizeof(double));
	delete [] inputs_storage;
	inputs_storage = tmp;
	n_inputs_alloc += 5;
    }
    
    for (size_t i=0; i<3; i++)
	inputs_storage[n_inputs * 3 + i] = input[i];

    n_inputs++;

    return true;
}

void yarp::sig::FilterCommand::setTag(int tag)
{
    this->tag_value = tag;
}

void yarp::sig::FilterCommand::setCommand(int cmd)
{
    this->cmd_value = cmd;
}

void yarp::sig::FilterCommand::points(std::vector<yarp::sig::Vector>& points) const
{
    points.clear();

    for(size_t i=0; i<n_points; i++)
    {
	yarp::sig::Vector point(3, &(points_storage[i * 3]));
	points.push_back(point);
    }
}

void yarp::sig::FilterCommand::inputs(std::vector<yarp::sig::Vector>& inputs) const
{
    inputs.clear();

    for(size_t i=0; i<n_inputs; i++)
    {
	yarp::sig::Vector input(3, &(inputs_storage[i * 3]));
	inputs.push_back(input);
    }
}

int yarp::sig::FilterCommand::tag() const
{
    return tag_value;
}

int yarp::sig::FilterCommand::command() const
{
    return cmd_value;
}

void yarp::sig::FilterCommand::clear()
{
    n_points = 0;
    n_inputs = 0;
    tag_value = VOCAB4('E','M','P','T');
    cmd_value = VOCAB4('E','M','P','T');
}

bool yarp::sig::FilterCommand::read(yarp::os::ConnectionReader& connection)
{
    FilterCommandPortContentHeader header;

    bool ok = connection.expectBlock((char*)&header, sizeof(header));
    if (!ok)
	return false;

    // set sizes
    n_points_alloc = n_points = header.n_points;
    n_inputs_alloc = n_inputs = header.n_inputs;

    // get command
    cmd_value = connection.expectInt();

    // get tag
    tag_value = connection.expectInt();

    // allocate points and set values
    points_storage = new double[3 * n_points_alloc];
    for (size_t i=0; i<n_points; i++)
	for (size_t j=0; j<3; j++)
	    points_storage[i*3 + j] = connection.expectDouble();

    // allocate inputs and set to zero
    inputs_storage = new double[3 * n_inputs_alloc];
    for (size_t i=0; i<n_inputs; i++)
	for (size_t j=0; j<3; j++)
	    inputs_storage[i*3 + j] = connection.expectDouble();

    return true;
}

bool yarp::sig::FilterCommand::write(yarp::os::ConnectionWriter& connection)
{
    FilterCommandPortContentHeader header;

    header.n_points = n_points;
    header.n_inputs = n_inputs;

    connection.appendBlock((char*)&header, sizeof(header));

    // append command
    connection.appendInt(cmd_value);

    // append tag
    connection.appendInt(tag_value);

    // append measures
    for (size_t i=0; i<n_points; i++)
	for (size_t j=0; j<3; j++)
	    connection.appendDouble(points_storage[i*3 + j]);

    // append inputs
    for (size_t i=0; i<n_inputs; i++)
	for (size_t j=0; j<3; j++)
	    connection.appendDouble(inputs_storage[i*3 + j]);

    return true;
}
