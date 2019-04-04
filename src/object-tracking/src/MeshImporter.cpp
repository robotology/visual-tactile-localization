/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MeshImporter.h>

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

using namespace Assimp;

MeshImporter::MeshImporter(const std::string& mesh_filename)
{
    Importer importer;

    importer.ReadFile(mesh_filename, aiProcessPreset_TargetRealtime_Fast);
    // Unless GetOrphanedScene is called, importer maintains the ownership
    // on the aiScene resulting in deallocation at the end of the ctor
    // when importer goes out of scope
    mesh_ = std::unique_ptr<aiScene>(importer.GetOrphanedScene());

    if (mesh_ == nullptr)
    {
        std::string err = "MESHIMPORTER::CTOR::ERROR\n\tError: cannot load mesh file " + mesh_filename + ".";
        throw(std::runtime_error(err));
    }
}


std::pair<bool, std::istringstream> MeshImporter::getMesh(const std::string& format_id)
{
    Exporter exporter;

    const aiExportDataBlob* blob;
    blob = exporter.ExportToBlob(mesh_.get(), format_id);

    if ((blob == nullptr) || (blob->size == 0))
        return std::make_pair(false, std::istringstream());

    return std::make_pair(true, std::istringstream(std::string(static_cast<const char*>(blob->data), blob->size)));
}
