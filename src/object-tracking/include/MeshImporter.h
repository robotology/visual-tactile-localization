#ifndef MESHIMPORTER_H
#define MESHIMPORTER_H

#include <assimp/scene.h>

#include <memory>
#include <sstream>
#include <string>


class MeshImporter
{
public:
    MeshImporter(const std::string& mesh_filename);

    std::pair<bool, std::istringstream> getMesh(const std::string& format_id);

protected:
    std::unique_ptr<aiScene> mesh_;
};

#endif /* MESHIMPORTER_H */
