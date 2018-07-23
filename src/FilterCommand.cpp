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
#include <yarp/os/Vocab.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>

#include "headers/FilterCommand.h"

void yarp::sig::FilterCommand::setTag(int tag)
{
    this->tag_value = tag;
}

void yarp::sig::FilterCommand::setCommand(int cmd)
{
    this->cmd_value = cmd;
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
    tag_value = yarp::os::createVocab('E','M','P','T');
    cmd_value = yarp::os::createVocab('E','M','P','T');
}

bool yarp::sig::FilterCommand::read(yarp::os::ConnectionReader& connection)
{
    // TODO: add checks

    // get command
    cmd_value = connection.expectInt();

    // get tag
    tag_value = connection.expectInt();

    return true;
}

bool yarp::sig::FilterCommand::write(yarp::os::ConnectionWriter& connection) const
{
    // append command
    connection.appendInt(cmd_value);

    // append tag
    connection.appendInt(tag_value);

    return true;
}
