/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef SCRATCH_DATASET_TILES_H
#define SCRATCH_DATASET_TILES_H

#include <osg/Group>
#include <osg/Camera>

class DataSetTiles
{
public:
    virtual ~DataSetTiles() {}
    virtual void Update(osg::Camera const * camera)=0;
};

#endif // SCRATCH_DATASET_TILES_H
