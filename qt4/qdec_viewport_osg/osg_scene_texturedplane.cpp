    // draw a plane
    osg::ref_ptr<osg::Vec3Array> listVxArray = new osg::Vec3Array;
    listVxArray->push_back(osg::Vec3(-1,0,1));  // top left
    listVxArray->push_back(osg::Vec3(-1,0,-1)); // bottom left
    listVxArray->push_back(osg::Vec3(1,0,-1));  // bottom right
    listVxArray->push_back(osg::Vec3(1,0,1));   // top right

    osg::ref_ptr<osg::Vec2Array> listTxArray = new osg::Vec2Array;
    listTxArray->push_back(osg::Vec2(0,1));     // top left
    listTxArray->push_back(osg::Vec2(0,0));     // bottom left
    listTxArray->push_back(osg::Vec2(1,0));     // bottom right
    listTxArray->push_back(osg::Vec2(1,1));     // top right

    osg::ref_ptr<osg::DrawElementsUInt> listIdxs =
            new osg::DrawElementsUInt(GL_TRIANGLES,6);

    listIdxs->at(0) = 0;
    listIdxs->at(1) = 1;
    listIdxs->at(2) = 2;

    listIdxs->at(3) = 0;
    listIdxs->at(4) = 2;
    listIdxs->at(5) = 3;

    // load texture
    QString imgPath = m_resPrefix + "/home/preet/Pictures/random/harle_from_chrono_cross.jpg";
    osg::ref_ptr<osg::Image> texImg = osgDB::readImageFile(imgPath.toStdString());
    osg::ref_ptr<osg::Texture2D> texGeom = new osg::Texture2D;
    texGeom->setImage(texImg);
    texGeom->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
    texGeom->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);
    texGeom->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
    texGeom->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);

    // setup geometry
    osg::ref_ptr<osg::Geometry> geomMesh = new osg::Geometry;
    geomMesh->setVertexArray(listVxArray);
    geomMesh->setTexCoordArray(1,listTxArray);
    geomMesh->addPrimitiveSet(listIdxs);
    geomMesh->setUseVertexBufferObjects(true);
    geomMesh->getOrCreateStateSet()->setTextureAttributeAndModes(1,texGeom);

    // node mesh
    osg::ref_ptr<osg::Geode> geodeMesh = new osg::Geode;
    geodeMesh->addDrawable(geomMesh);

    // setup shaders
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;
    shProgram->setName("TextShader");

    QString vShader = readFileAsQString(m_resPrefix + "shaders/DirectTexture_vert.glsl");
    vShader.prepend(m_shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));

    QString fShader = readFileAsQString(m_resPrefix + "shaders/DirectTexture_frag.glsl");
    vShader.prepend(m_shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    osg::ref_ptr<osg::Uniform> myColor = new osg::Uniform("Color",osg::Vec4(0,1,1,1));
    osg::ref_ptr<osg::Uniform> myTexture = new osg::Uniform("Texture",1);

    // enable shader program
    osg::StateSet *ss = geodeMesh->getOrCreateStateSet();
    ss->addUniform(myColor);
    ss->addUniform(myTexture);
    ss->setAttributeAndModes(shProgram,osg::StateAttribute::ON);
    ss->setMode(GL_BLEND,osg::StateAttribute::ON);

    // add stuff to scene
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(geodeMesh);

    // create viewer and embedded window
    m_osg_viewer = new osgViewer::Viewer;
    m_osg_window = m_osg_viewer->setUpViewerAsEmbeddedInWindow(0,0,this->width(),this->height());

    // tell osg to insert uniforms and attributes in shaders
    m_osg_window->getState()->setUseModelViewAndProjectionUniforms(true);
    m_osg_window->getState()->setUseVertexAttributeAliasing(true);

    // config viewer
    m_osg_viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    m_osg_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    m_osg_viewer->setSceneData(groupRoot);
    m_osg_viewer->realize();

    // setup state
    m_osg_stateset = new osg::StateSet;

//    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::ONCE_PER_ATTRIBUTE);

    // setup camera
//    m_osg_viewer->getCamera()->setClearColor(osg::Vec4(0,0,0,0));
    m_osg_viewer->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT);
