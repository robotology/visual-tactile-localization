#ifndef CONTACTDETECTION_H
#define CONTACTDETECTION_H

#include <unordered_map>

class ContactDetection
{
public:
    virtual std::pair<bool, std::unordered_map<std::string, bool>> getActiveFingers() = 0;
};

#endif /* CONTACTDETECTION_H */
