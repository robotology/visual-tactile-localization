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
#include <yarp/sig/Vector.h>
#include <yarp/os/Portable.h>

// std
#include <vector>

namespace yarp {
    namespace sig {
	class FilterData;
    }
}

class YARP_sig_API yarp::sig::FilterData : public yarp::os::Portable
{
private:
    /*
     * Storage for the measurements
     */
    double *points_storage;
    int n_points_alloc;
    int n_points;
    
    /*
     * Storage for the inputs
     */
    double *inputs_storage;
    int n_inputs_alloc;
    int n_inputs;
    
    /*
     * Tag describing the type of FilterData, i.e.
     * something meaningful for the filtering algorithm
     */
    int tag_value;

    /*
     * Command describing something meaningful for the
     * filtering algorithm, e.g., start/stop commands
     */
    int cmd_value;

public:
    FilterData();
    FilterData(const FilterData &);    
    ~FilterData();

    FilterData& operator=(const FilterData&);
    
    bool addPoint(const yarp::sig::Vector&);
    bool addInput(const yarp::sig::Vector&);
    void setTag(int);
    void setCommand(int);

    void points(std::vector<yarp::sig::Vector>&) const;
    void inputs(std::vector<yarp::sig::Vector>&) const;
    int tag() const;
    int command() const;

    void clear();

    /*
     * Read a FilterData from a connection.
     * Return true iff a FilterData was read correctly.
     */
    bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE;
    
    bool write(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE;
};
#endif
