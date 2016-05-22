//==============================================================================
/*
    Software License Agreement ( BSD License )
    Copyright ( c ) 2003-2014, CHAI3D.
    ( www.chai3d.org )

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES ( INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION ) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT ( INCLUDING NEGLIGENCE OR OTHERWISE ) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.0.0 $Rev: 1292 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include <vector>
#include <time.h>
#include <ctime>
#include "chai3d.h"
#include "tinyxml2.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
using namespace tinyxml2;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = true;

// mirrored display
bool mirroredDisplay = false;

const float NUCLEUS_RADIUS = 0.1;
const float NUCLEUS_HEIGHT = 0.03;

const float TORUS_INNER = 0.003;
const float TORUS_OUTER = 0.2;
const float TORUS_DISTANCE = 0.1;
const float INNER_THRESHOLD = 0.001;
const float OUTER_THRESHOLD = 0.05;

const float SPRING_CONSTANT = 500;

const float PARTICLE_BOX = 0.17;
const float PARTICLE_BOX_HEIGHT = 0.03;

const float SELECTED_PARTICLE_RADIUS = 0.01;

const int NUM_SHELLS = 2;
const int NUM_PARTICLE_TYPE = 3;
const int ELECTRON = 0;
const int PROTON = 1;
const int NEUTRON = 2;

const int NUM_ATOMS = 6;

const double CLEAR_SCREEN_TIMEOUT = 0.15;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// rotational velocity of the object
cVector3d rotVel;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// root resource path
string resourceRoot;

// display level for collision tree
int collisionTreeDisplayLevel = 0;

// Atom nucleus
cShapeTorus* shells[NUM_SHELLS];
cShapeCylinder* nucleus;
cShapeBox* particle_boxes[NUM_PARTICLE_TYPE];
cShapeBox* chosen_atom;
cLabel* atom_num;
cLabel* atom_label;
cLabel* atom_name;
cLabel* particle_labels[NUM_PARTICLE_TYPE];
vector<cShapeSphere*> placed_atoms;

// Selected atom particle
cShapeSphere* selected_particle;
int is_selected = -1;
int in_ok_position = -1;

// Selected_particle material
cMaterialPtr select_material[NUM_PARTICLE_TYPE];

// The current atom
int current_atom_num;
string current_atom_symbol;
string current_atom_name;

// XML
XMLDocument doc;
XMLElement* pRoot;

// Game rules
int particles_left[NUM_PARTICLE_TYPE];
cLabel* clear_screen;
bool show_clear_screen = false;
clock_t timeout_start;

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH( p )    ( char* )( ( resourceRoot+string( p ) ).c_str() )


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow( int w, int h );

// callback when a key is pressed
void keySelect( unsigned char key, int x, int y );

// callback to render graphic scene
void updateGraphics( void );

// callback of GLUT timer
void graphicsTimer( int data );

// function that closes the application
void close( void );

// main haptics simulation loop
void updateHaptics( void );

// searches for an atom with a given number and sets it as the current atom
void getNewAtom( int );

// Removes all placed particles from screen
void resetScreen( void );

//==============================================================================
/*
    DEMO:    object.cpp

    This demonstration loads a 3D mesh file by using the file loader
    functionality of the cMesh class. A finger-proxy algorithm is used to
    render the forces. Take a look at this example to understand the
    different functionalities offered by the tool force renderer.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the device.
    Finally, a simple dynamics model is used to simulate the behavior
    of the object.
*/
//==============================================================================

int main( int argc, char* argv[] )
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "DH2626 - Lab 2" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Get new atom" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string( argv[0] ).substr( 0,string( argv[0] ).find_last_of( "/\\" )+1 );

    srand( time( NULL ) );


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit( &argc, argv );

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet( GLUT_SCREEN_WIDTH );
    screenH = glutGet( GLUT_SCREEN_HEIGHT );
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = ( screenH - windowH ) / 2;
    windowPosX = windowPosY;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition( windowPosX, windowPosY );
    glutInitWindowSize( windowW, windowH );
    if ( stereoMode == C_STEREO_ACTIVE )
    {
        glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO );
    }
    else
    {
        glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE );
    }

    // create display context and initialize GLEW library
    glutCreateWindow( argv[0] );
    glewInit();
    // setup GLUT options
    glutDisplayFunc( updateGraphics );
    glutKeyboardFunc( keySelect );
    glutReshapeFunc( resizeWindow );
    glutSetWindowTitle( "CHAI3D" );

    // set fullscreen mode
    if ( fullscreen )
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera( world );
    world->addChild( camera );

    // position and orient the camera
    camera->set( cVector3d ( 2, 0.0, 0 ),    // camera position ( eye )
                 cVector3d ( 0.0, 0.0, 0.0 ),    // lookat position ( target )
                 cVector3d ( 0.0, 0.0, 1.0 ) );   // direction of the ( up ) vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes( 0.01, 10.0 );

    // Set orthogonalic view
    camera->setOrthographicView( 1.3 );

    // set stereo eye separation and focal length ( applies only if stereo is enabled )
    camera->setStereoEyeSeparation( 0.03 );
    camera->setStereoFocalLength( 1.5 );

    // set vertical mirrored display mode
    camera->setMirrorVertical( mirroredDisplay );

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency( true );

    // create a light source
    light = new cDirectionalLight( world );

    // attach light to camera
    camera->addChild( light );

    // enable light source
    light->setEnabled( true );

    // define the direction of the light beam
    light->setDir( 0.0,0.0, 0.0 );

    // set lighting conditions
    light->m_ambient.set( 1.0, 1.0, 1.0 );
    light->m_diffuse.set( 0.8, 0.8, 0.8 );
    light->m_specular.set( 0.0, 0.0, 0.0 );


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice( hapticDevice, 0 );

    // retrieve information about the current haptic device
    //cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool ( cursor ) and insert into the world
    tool = new cToolCursor( world );
    world->addChild( tool );

    // connect the haptic device to the virtual tool
    tool->setHapticDevice( hapticDevice );

    // define the radius of the tool ( sphere )
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius( toolRadius );

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints( true, false );

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects( true );

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius( 0.4 );

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE OBJECTS
    //--------------------------------------------------------------------------
    cMatrix3d rot;
    rot.identity();
    rot.rotateAboutGlobalAxisDeg( cVector3d( 0,1,0 ), 90 );
    cMatrix3d rot1;
    rot1.identity();
    rot1.rotateAboutGlobalAxisDeg( cVector3d( 0,1,0 ), 57 );

    /*
     * Selected particle materials
     */
    for ( int i = 0; i < NUM_PARTICLE_TYPE; i++ )
    {
        select_material[i] = cMaterialPtr( new cMaterial() );
        is_selected = -1;
    }
    select_material[ELECTRON]->setBlue();
    select_material[PROTON]->setOrangeRed();
    select_material[NEUTRON]->setGray();

    /*
     * Nucleus cylinder
     */
    cMaterialPtr nucleus_mat;
    cMaterial* c = new cMaterial();
    c->setBlack();
    nucleus_mat = cMaterialPtr( c );

    nucleus = new cShapeCylinder( NUCLEUS_RADIUS, NUCLEUS_RADIUS, NUCLEUS_HEIGHT, nucleus_mat );
    nucleus->setLocalRot( rot );
    world->addChild( nucleus );
    nucleus->setLocalPos( 0.0,0.0,0.0 );

    /*
     * Shell toruses
     */
    cMaterialPtr torus_mat = cMaterialPtr( new cMaterial() );
    torus_mat->setBlack();

    for ( int i = 0; i < NUM_SHELLS; i++ )
    {
        shells[i] = new cShapeTorus( TORUS_INNER, TORUS_OUTER + i*TORUS_DISTANCE, torus_mat );
        shells[i]->setLocalRot( rot );
        world->addChild( shells[i] );

        shells[i]->setLocalPos( 0.0,0.0,0.0 );

    }

    /*
     * Electron, Proton and Neutron particle boxes
     */
    for ( int i = 0; i < NUM_PARTICLE_TYPE; i++ )
    {
        particle_boxes[i] = new cShapeBox( PARTICLE_BOX, PARTICLE_BOX, PARTICLE_BOX_HEIGHT, select_material[i] );
        particle_boxes[i]->setLocalRot( rot1 );
        world->addChild( particle_boxes[i] );
        particle_boxes[i]->setLocalPos( 0.0,0.45,0.25-i*0.25 );
    }

    /*
     * Chosen atom box
     */
    cMaterialPtr chosen_atom_mat;
    cMaterial* c_chosen = new cMaterial();
    c_chosen->setYellow();
    chosen_atom_mat = cMaterialPtr( c_chosen );
    chosen_atom = new cShapeBox( PARTICLE_BOX, PARTICLE_BOX, PARTICLE_BOX_HEIGHT,chosen_atom_mat );
    chosen_atom->setLocalRot( rot1 );
    world->addChild( chosen_atom );
    chosen_atom->setLocalPos( 0.0,-0.48,0.28 );

    /*
     * Selected particle sphere
     */
    cMaterialPtr selected_particle_mat;
    cMaterial* c_selected = new cMaterial();
    c_selected->setBlue();
    selected_particle_mat = cMaterialPtr( c_selected );
    selected_particle = new cShapeSphere( SELECTED_PARTICLE_RADIUS, selected_particle_mat );
    world->addChild( selected_particle );

    /*
     * XML
     */
    getNewAtom( 1 );
    //getNewAtom( 4 );

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI32();
    cFont *font_name = NEW_CFONTCALIBRI36();
    cFont *font_label = NEW_CFONTCALIBRI144();

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel( font );
    labelHapticRate->m_fontColor.setBlack();
    camera->m_frontLayer->addChild( labelHapticRate );

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild( background );

    // set background properties
    background->setCornerColors( cColorf( 1.0, 1.0, 1.0 ),
                                cColorf( 1.0, 1.0, 1.0 ),
                                cColorf( 0.8, 0.8, 0.8 ),
                                cColorf( 0.8, 0.8, 0.8 ) );

    atom_num = new cLabel( font );
    atom_num->m_fontColor.setBlack();
    camera->m_frontLayer->addChild( atom_num );

    atom_label = new cLabel( font_label );
    atom_label->m_fontColor.setBlack();
    camera->m_frontLayer->addChild( atom_label );

    atom_name = new cLabel( font_name );
    atom_name->m_fontColor.setBlack();
    camera->m_frontLayer->addChild( atom_name );

    for ( int i = 0; i < NUM_PARTICLE_TYPE; i++ )
    {
        particle_labels[i] = new cLabel( font_name );
        particle_labels[i]->m_fontColor.setBlack();
        camera->m_frontLayer->addChild( particle_labels[i] );
    }

    clear_screen = new cLabel( font_label );
    clear_screen->m_fontColor.setGreen();
    clear_screen->setString( "WELL DONE" );

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start( updateHaptics, CTHREAD_PRIORITY_HAPTICS );

    // start the main graphics rendering loop
    glutTimerFunc( 50, graphicsTimer, 0 );
    glutMainLoop();

    // close everything
    close();

    // exit
    return ( 0 );
}

//------------------------------------------------------------------------------

void resizeWindow( int w, int h )
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void getNewAtom( int num )
{
    doc.LoadFile( "atoms.xml" );
    pRoot = doc.FirstChildElement( "atoms" );
    XMLElement* newAtom = NULL;

    for ( XMLElement* child = pRoot->FirstChildElement(); child != NULL; child = child->NextSiblingElement() )
    {
        string current_num = child->FirstChildElement( "num" )->GetText();
        if ( atoi( current_num.c_str() ) == num )
        {
            newAtom = child;
            break;
        }
    }

    if ( newAtom != NULL )
    {
        string atom_num_str = newAtom->FirstChildElement( "num" )->GetText();

        current_atom_num = atoi( atom_num_str.c_str() );
        current_atom_symbol = newAtom->FirstChildElement( "symbol" )->GetText();
        current_atom_name = newAtom->FirstChildElement( "name" )->GetText();
    }
    // Update game rules
    for ( int i = 0; i < NUM_PARTICLE_TYPE; i++ )
    {
        particles_left[i] = current_atom_num;
    }

    // Special case for hydrogen
    if ( current_atom_num == 1 )
    {
        particles_left[NEUTRON] = 0;
    }
}

//------------------------------------------------------------------------------

void resetScreen()
{
    while ( !placed_atoms.empty() )
    {
        world->removeChild( placed_atoms.back() );
        delete placed_atoms.back();
        placed_atoms.pop_back();
    }
}

//------------------------------------------------------------------------------

void keySelect( unsigned char key, int x, int y )
{
    // option ESC: exit
    if ( ( key == 27 ) || ( key == 'x' ) )
    {
        // close everything
        close();

        // exit application
        exit( 0 );
    }

    // option 1: Select a random atom
    if ( key == '1' )
    {
        // Reset screen
        resetScreen();

        // Get new random atom
        getNewAtom( rand() % NUM_ATOMS + 1 );
    }

    // option f: toggle fullscreen
    if ( key == 'f' )
    {
        if ( fullscreen )
        {
            windowPosX = glutGet( GLUT_INIT_WINDOW_X );
            windowPosY = glutGet( GLUT_INIT_WINDOW_Y );
            windowW = glutGet( GLUT_INIT_WINDOW_WIDTH );
            windowH = glutGet( GLUT_INIT_WINDOW_HEIGHT );
            glutPositionWindow( windowPosX, windowPosY );
            glutReshapeWindow( windowW, windowH );
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if ( key == 'm' )
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical( mirroredDisplay );
    }
}

//------------------------------------------------------------------------------

void close( void )
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while ( !simulationFinished ) { cSleepMs( 100 ); }

    // close haptic device
    tool->stop();
}

//------------------------------------------------------------------------------

void graphicsTimer( int data )
{
    if ( simulationRunning )
    {
        glutPostRedisplay();
    }

    glutTimerFunc( 50, graphicsTimer, 0 );
}

//------------------------------------------------------------------------------

void updateGraphics( void )
{
    /////////////////////////////////////////////////////////////////////
    // CHECK GAME STATUS
    /////////////////////////////////////////////////////////////////////

    if ( particles_left[ELECTRON] == 0 &&
        particles_left[PROTON] == 0 &&
        particles_left[NEUTRON] == 0 )
    {
        // Display game over screen
        camera->m_frontLayer->addChild( clear_screen );
        // Set timeout
        timeout_start = clock();

        show_clear_screen = true;

        // Get new random atom
        getNewAtom( rand() % NUM_ATOMS + 1 );
    }
    // If timeout
    if ( show_clear_screen && ( ( clock() - timeout_start )/( double )CLOCKS_PER_SEC ) > CLEAR_SCREEN_TIMEOUT )
    {
        // Reset screen
        resetScreen();
        show_clear_screen = false;
        camera->m_frontLayer->removeChild( clear_screen );
    }

    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic rate label
    labelHapticRate->setString( "haptic rate: "+cStr( frequencyCounter.getFrequency(), 0 ) + " [Hz]" );

    // update position of label
    labelHapticRate->setLocalPos( ( int )( 0.5 * ( windowW - labelHapticRate->getWidth() ) ), 15 );

    // Set text and position of atom number label
    atom_num->setString( to_string( current_atom_num ) );
    atom_num->setLocalPos( ( int )( 0.089 * ( windowW - atom_num->getWidth() ) ),
                          ( int )( 0.93 * ( windowH - atom_num->getHeight() ) ) );

    // Set text and position of element symbol label
    atom_label->setString( current_atom_symbol );
    atom_label->setLocalPos( ( int )( 0.11 * ( windowW - atom_label->getWidth() ) ),
                            ( int )( 0.91 * ( windowH - atom_label->getHeight() ) ) );

    // Set text and position of element name label
    atom_name->setString( current_atom_name );
    atom_name->setLocalPos( ( int )( 0.08 * ( windowW - atom_name->getWidth() ) ),
                           ( int )( 0.81 * ( windowH - atom_name->getHeight() ) ) );

    // Set text and posistion of particle labels
    particle_labels[ELECTRON]->setString( "e-" );
    particle_labels[PROTON]->setString( "p+" );
    particle_labels[NEUTRON]->setString( "n" );
    particle_labels[ELECTRON]->setLocalPos( ( int )( 0.95 * ( windowW - particle_labels[0]->getWidth() ) ),
                                            ( int )( 0.835 * ( windowH - particle_labels[0]->getHeight() ) ) );
    particle_labels[PROTON]->setLocalPos( ( int )( 0.95 * ( windowW - particle_labels[1]->getWidth() ) ),
                                          ( int )( 0.520 * ( windowH - particle_labels[1]->getHeight() ) ) );
    particle_labels[NEUTRON]->setLocalPos( ( int )( 0.95 * ( windowW - particle_labels[2]->getWidth() ) ),
                                           ( int )( 0.190 * ( windowH - particle_labels[2]->getHeight() ) ) );

    // Check if button is pressed
    bool buttonStatus;
    hapticDevice->getUserSwitch( 0, buttonStatus );
    if ( buttonStatus && !show_clear_screen )
    {
        // If user is holding the button, don't change color
        if ( !selected_particle->getShowEnabled() )
        {
            // Get cursor position
            cVector3d tool_pos;
            tool_pos = tool->getDeviceLocalPos();
            for ( int i = 0; i < NUM_PARTICLE_TYPE; i++ )
            {
                // Get box position
                cVector3d boxPos = particle_boxes[i]->getLocalPos();
                double box_x_max = boxPos.x() + PARTICLE_BOX*2; // Let user pick above box
                double box_x_min = boxPos.x() - PARTICLE_BOX*2; // Let user pick above box
                double box_y_max = boxPos.y() + PARTICLE_BOX/2;
                double box_y_min = boxPos.y() - PARTICLE_BOX/2;
                double box_z_max = boxPos.z() + PARTICLE_BOX/2;
                double box_z_min = boxPos.z() - PARTICLE_BOX/2;

                // If collision, set colors
                if ( tool_pos.x() <= box_x_max && tool_pos.x() >= box_x_min &&
                     tool_pos.y() <= box_y_max && tool_pos.y() >= box_y_min &&
                     tool_pos.z() <= box_z_max && tool_pos.z() >= box_z_min )
                {
                    selected_particle->setMaterial( select_material[i] );
                    selected_particle->setShowEnabled( true,true );
                    is_selected = i;
                }
            }
        }
    }
    else
    {
        // If user released the button with a selected particle
        if ( ELECTRON == is_selected && ELECTRON == in_ok_position && particles_left[ELECTRON] > 0 )
        {
            // Place it on the screen
            cShapeSphere* placed_atom = selected_particle->copy();
            world->addChild( placed_atom );
            placed_atoms.push_back( placed_atom );

            particles_left[ELECTRON]--;

            in_ok_position = -1;
        }
        if ( PROTON == is_selected && PROTON == in_ok_position && particles_left[PROTON] > 0 )
        {
            // Place it on the screen
            cShapeSphere* placed_atom = selected_particle->copy();
            world->addChild( placed_atom );
            placed_atoms.push_back( placed_atom );

            particles_left[PROTON]--;

            in_ok_position = -1;
        }
        if ( NEUTRON == is_selected && PROTON == in_ok_position && particles_left[NEUTRON] > 0 )
        {
            // Place it on the screen
            cShapeSphere* placed_atom = selected_particle->copy();
            world->addChild( placed_atom );
            placed_atoms.push_back( placed_atom );

            particles_left[NEUTRON]--;

            in_ok_position = -1;
        }
        selected_particle->setShowEnabled( false, false );
        is_selected = -1;
    }



    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // render world
    camera->renderView( windowW, windowH );

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if ( err != GL_NO_ERROR ) cout << "Error: " << gluErrorString( err ) << endl;
}

//------------------------------------------------------------------------------

enum cMode
{
    IDLE,
    SELECTION
};

void updateHaptics( void )
{

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while( simulationRunning )
    {

        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // update frequency counter
        frequencyCounter.signal( 1 );

        // compute global reference frames for each object
        world->computeGlobalPositions( true );

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // update selected particle to cursor
        cVector3d proxy_pos, tool_pos;
        tool_pos = tool->getDeviceLocalPos();
        proxy_pos.x( tool_pos.x() - SELECTED_PARTICLE_RADIUS );
        proxy_pos.y( tool_pos.y() - SELECTED_PARTICLE_RADIUS );
        proxy_pos.z( tool_pos.z() + SELECTED_PARTICLE_RADIUS );
        selected_particle->setLocalPos( proxy_pos );

        cVector3d x_vector( 0, 0, 0 );
        x_vector.x( proxy_pos.x() );

        // temp_vector is on the y-z plane
        cVector3d temp_vector = proxy_pos - x_vector;
        cVector3d unit_vector = temp_vector;
        unit_vector.normalize();

        cVector3d force( 0, 0, 0 );
        
        // If user has picked up a electron, set magnetical effect around shells
        if ( ELECTRON == is_selected )
        {
            for ( int i = 0; i < NUM_SHELLS; i++ )
            {
                // Set force field depending on the number of electrons placed
                if ( particles_left[ELECTRON] == 0 )
                {
                    break;
                }
                // If less than two electrons are placed, skip force for outer shells
                // If at least two electrons are placed, skip first shell
                if ( current_atom_num - particles_left[ELECTRON] < 2 && i > 0 ||
                     current_atom_num - particles_left[ELECTRON] >= 2 && i == 0 )
                {
                    continue;
                }
                // If we are within the force field
                if( temp_vector.length() < ( TORUS_OUTER + OUTER_THRESHOLD + TORUS_DISTANCE*i ) &&
                   temp_vector.length() > ( TORUS_OUTER - OUTER_THRESHOLD + TORUS_DISTANCE*i ) &&
                   proxy_pos.x() < 0.07 )
                {
                    in_ok_position = ELECTRON;
                    // If we are close to the center of the torus
                    if ( temp_vector.length() >= ( TORUS_OUTER + INNER_THRESHOLD + TORUS_DISTANCE*i ) ||
                        temp_vector.length() <= ( TORUS_OUTER - INNER_THRESHOLD + TORUS_DISTANCE*i ) )
                    {
                        // We are either inside the inner or outer force field
                        if ( temp_vector.length() > ( TORUS_OUTER - OUTER_THRESHOLD + TORUS_DISTANCE*i ) &&
                            temp_vector.length() <= ( TORUS_OUTER - INNER_THRESHOLD + TORUS_DISTANCE*i ) )
                        {
                            // When on the inside force field for the torus
                            force = SPRING_CONSTANT * ( unit_vector*( TORUS_OUTER - INNER_THRESHOLD + TORUS_DISTANCE*i ) - temp_vector );
                        }
                        else {
                            // When on the outer force field for the torus
                            force = SPRING_CONSTANT * ( unit_vector*( TORUS_OUTER + INNER_THRESHOLD + TORUS_DISTANCE*i ) - temp_vector );
                        }
                    }
                    break;
                }
                else {
                    in_ok_position = -1;
                }
            }

            // Prevent the user place an electron in the nucleus
            if ( temp_vector.length() <= NUCLEUS_RADIUS )
            {
                force = SPRING_CONSTANT * ( unit_vector*NUCLEUS_RADIUS - temp_vector );
            }
        }
        else if ( PROTON == is_selected || NEUTRON == is_selected )
        {
            // If cursor is close to the nucleus in the y-z plane, set force to centrum
            if ( temp_vector.length() < ( NUCLEUS_RADIUS + 0.05 ) &&
                proxy_pos.x() < 0.07 &&
                !( PROTON == is_selected && particles_left[PROTON] == 0 ) &&
                !( NEUTRON == is_selected && particles_left[NEUTRON] == 0 ) )
            {
                in_ok_position = PROTON;
                if ( temp_vector.length() >= NUCLEUS_RADIUS )
                {
                    force = SPRING_CONSTANT * ( unit_vector*( NUCLEUS_RADIUS ) - temp_vector );
                }
                // Cannot push inwards
                if ( proxy_pos.x() < 0.03 )
                {
                    force.x( SPRING_CONSTANT*( -proxy_pos.x() + 0.03 ) );
                }
                // Smaller force when pulling
                else if ( proxy_pos.x() > 0.04 )
                {
                    force.x( -500*( proxy_pos.x() - 0.04 ) );
                }
            }
            else {
                in_ok_position = -1;
            }
        }
        // send forces to haptic device
        tool->getHapticDevice()->setForce( force );
    }

    // exit haptics thread
    simulationFinished = true;
}
