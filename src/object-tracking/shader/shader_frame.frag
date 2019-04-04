/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#version 330 core

in vec3 vert_color;

out vec4 frag_color;

void main()
{
    frag_color = vec4(vert_color, 1.0f);
}
