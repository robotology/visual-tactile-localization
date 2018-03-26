/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef FILTER_DATA_H
#define FILTER_DATA_H

// yarp
#include <yarp/os/Portable.h>
#include <yarp/sig/api.h>

namespace yarp {
    namespace sig {
	class FilterCommand;
    }
}

class YARP_sig_API yarp::sig::FilterCommand : public yarp::os::Portable
{
private:
    /*
     * Tag describing the type of FilterCommand, i.e.
     * something meaningful for the filtering algorithm
     */
    int tag_value;

    /*
     * Command describing something meaningful for the
     * filtering algorithm, e.g., start/stop commands
     */
    int cmd_value;

public:
    void setTag(int);
    void setCommand(int);

    int tag() const;
    int command() const;

    void clear();

    /*
     * Read a FilterCommand from a connection.
     * Return true iff a FilterCommand was read correctly.
     */
    bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE;
    
    bool write(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE;
};
#endif
