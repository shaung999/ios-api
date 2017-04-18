// Copyright eeGeo Ltd (2012-2014), All Rights Reserved

#include "App.h"
#include "AppHost.h"
#include "LatLongAltitude.h"
#include "EegeoWorld.h"
#include "RenderContext.h"
#include "AppInterface.h"
#include "EffectHandler.h"
#include "SearchServiceCredentials.h"
#include "GlobeCameraController.h"
#include "RenderCamera.h"
#include "CameraHelpers.h"
#include "PlatformConfig.h"
#include "iOSPlatformConfigBuilder.h"
#include "EegeoWorld.h"
#include "JpegLoader.h"
#include "iOSPlatformAbstractionModule.h"
#include "ScreenProperties.h"
#include "BuildingFootprintsModule.h"
#include "CollisionVisualizationModule.h"
#include "Collision.h"

namespace
{
    Eegeo::Modules::BuildingFootprintsModule* CreateBuildingFootprintsModule(Eegeo::EegeoWorld& world, const Eegeo::Modules::CollisionVisualizationModule& collisionVisualizationModule)
    {
        const Eegeo::BuildingFootprints::BuildingFootprintSelectionControllerConfig& buildingFootprintSelectionControllerConfig = Eegeo::Modules::BuildingFootprintsModule::MakeDefaultConfig();
        
        const Eegeo::Modules::IPlatformAbstractionModule& platformAbstractionModule = world.GetPlatformAbstractionModule();
        const Eegeo::Modules::Core::RenderingModule& renderingModule = world.GetRenderingModule();
        const Eegeo::Modules::Map::MapModule& mapModule = world.GetMapModule();
        const Eegeo::Modules::Map::Layers::BuildingStreamingModule& buildingStreamingModule = mapModule.GetBuildingStreamingModule();
        const Eegeo::Modules::Map::CoverageTreeModule& coverageTreeModule = mapModule.GetCoverageTreeModule();
        
        Eegeo::Modules::BuildingFootprintsModule* pBuildingFootprintsModule =
        Eegeo::Modules::BuildingFootprintsModule::Create(platformAbstractionModule,
                                                         renderingModule,
                                                         collisionVisualizationModule,
                                                         buildingStreamingModule,
                                                         coverageTreeModule,
                                                         buildingFootprintSelectionControllerConfig);
        return pBuildingFootprintsModule;
    }
    
    Eegeo::Modules::CollisionVisualizationModule* CreateCollisionVisualizationModule(Eegeo::EegeoWorld& world)
    {
        const Eegeo::CollisionVisualization::MaterialSelectionControllerConfig& materialSelectionControllerConfig = Eegeo::Modules::CollisionVisualizationModule::MakeDefaultConfig();
        
        
        const Eegeo::Modules::Core::RenderingModule& renderingModule = world.GetRenderingModule();
        const Eegeo::Modules::Map::MapModule& mapModule = world.GetMapModule();
        
        Eegeo::Modules::CollisionVisualizationModule* pCollisionVisualizationModule = Eegeo::Modules::CollisionVisualizationModule::Create(renderingModule, mapModule, materialSelectionControllerConfig);
        return pCollisionVisualizationModule;
    }
}

using namespace Eegeo::iOS;

AppHost::AppHost(const std::string& apiKey,
                 const Eegeo::Rendering::ScreenProperties& screenProperties)
    :m_pJpegLoader(NULL)
	,m_piOSLocationService(NULL)
	,m_pWorld(NULL)
	,m_iOSInputBoxFactory()
	,m_iOSKeyboardInputFactory()
	,m_iOSAlertBoxFactory()
	,m_iOSNativeUIFactories(m_iOSAlertBoxFactory, m_iOSInputBoxFactory, m_iOSKeyboardInputFactory)
    ,m_piOSPlatformAbstractionModule(NULL)
	,m_pApp(NULL)
    ,m_pAppInputDelegate(NULL)
{
	m_piOSLocationService = new iOSLocationService();
	   
    m_pJpegLoader = new Eegeo::Helpers::Jpeg::JpegLoader();
    
    m_piOSPlatformAbstractionModule = new Eegeo::iOS::iOSPlatformAbstractionModule(*m_pJpegLoader, apiKey);

	Eegeo::EffectHandler::Initialise();

	Eegeo::Config::PlatformConfig config = Eegeo::iOS::iOSPlatformConfigBuilder(App::GetDevice(), App::IsDeviceMultiCore(), App::GetMajorSystemVersion()).Build();
    
    config.OptionsConfig.StartMapModuleAutomatically = false;
    config.OptionsConfig.GenerateCollisionForAllResources = true;
    
    config.OptionsConfig.EnableLabels = true;
    
    const std::string defaultFont = "opensans_semibold_sdf.fnt";
    
    config.MapLayersConfig.FontsModuleConfig.EnvironmentFontFilename = defaultFont;
    config.MapLayersConfig.DebugRenderingModuleConfig.DebugFontFilename = defaultFont;
    
    config.MapLayersConfig.LabelsModuleConfig.StyleSheetPath = "Labels/label_style_sheet.json";
    config.MapLayersConfig.LabelsModuleConfig.CategoryIconMapPath = "Labels/label_category_icon_map.json";
    config.MapLayersConfig.Interiors.LabelCategoryMapPath = "Interiors/label_category_mapping.json";
    config.MapLayersConfig.IconsModuleConfig.IconsEnabled = true;
    config.MapLayersConfig.IconsModuleConfig.IconSetManifestPath = "pin_sheet.json";
    config.MapLayersConfig.Interiors.UseLegacyLabels = false;
    config.MapLayersConfig.Interiors.LabelFontTextureFilename = defaultFont;
    
    config.CityThemesConfig.StreamedManifestUrl = [[[NSUserDefaults standardUserDefaults] objectForKey:@"manifestUrl"] UTF8String];
    config.CoverageTreeConfig.ManifestUrl = [[[NSUserDefaults standardUserDefaults] objectForKey:@"coverageUrl"] UTF8String];

	m_pWorld = new Eegeo::EegeoWorld(apiKey,
                                     *m_piOSPlatformAbstractionModule,
                                     *m_pJpegLoader,
                                     screenProperties,
                                     *m_piOSLocationService,
                                     m_iOSNativeUIFactories,
                                     Eegeo::EnvironmentCharacterSet::UseFontModuleConfig,
                                     config,
                                     NULL);
    
    printf( "Manif Url: %s\n", config.CityThemesConfig.StreamedManifestUrl.c_str() );
    printf( "Embed Url: %s\n", config.CityThemesConfig.EmbeddedThemeManifestFile.c_str() );
    
    m_pCollisionVisualizationModule = CreateCollisionVisualizationModule(*m_pWorld);
    m_pBuildingFootprintsModule = CreateBuildingFootprintsModule(*m_pWorld, *m_pCollisionVisualizationModule);
    
    Eegeo::Space::LatLong currentLatLng(51.5073509,-0.1277583);
    m_piOSLocationService->GetLocation( currentLatLng );
    m_pApp = new ExampleApp(m_pWorld, currentLatLng, screenProperties, *m_pCollisionVisualizationModule, *m_pBuildingFootprintsModule);
    
    m_pAppLocationDelegate = new AppLocationDelegate(*m_piOSLocationService);
    
    Eegeo::TtyHandler::TtyEnabled = false;
}

AppHost::~AppHost()
{
    delete m_pAppLocationDelegate;
    m_pAppLocationDelegate = NULL;
    
	delete m_pAppInputDelegate;
	m_pAppInputDelegate = NULL;

	delete m_pApp;
	m_pApp = NULL;

	delete m_pWorld;
	m_pWorld = NULL;

	delete m_piOSLocationService;
	m_piOSLocationService = NULL;
    
    delete m_piOSPlatformAbstractionModule;
    m_piOSPlatformAbstractionModule = NULL;
    
    delete m_pJpegLoader;
    m_pJpegLoader = NULL;
    
    delete m_pBuildingFootprintsModule;
    m_pBuildingFootprintsModule = NULL;
    
    delete m_pCollisionVisualizationModule;
    m_pCollisionVisualizationModule = NULL;
    
	Eegeo::EffectHandler::Reset();
	Eegeo::EffectHandler::Shutdown();
}

void AppHost::UnbindInputProvider()
{
    Eegeo_DELETE m_pAppInputDelegate;
    m_pAppInputDelegate = NULL;
}

void AppHost::BindInputProvider(GLKView &view,
                                id<UIGestureRecognizerDelegate>& gestureRecognizer,
                                const Eegeo::Rendering::ScreenProperties& screenProperties)
{
    UnbindInputProvider();
    
    m_pAppInputDelegate = new AppInputDelegate(*m_pApp,
                                               view,
                                               gestureRecognizer,
                                               screenProperties.GetScreenWidth(),
                                               screenProperties.GetScreenHeight(),
                                               screenProperties.GetPixelScale());
}

void AppHost::BindApi(EegeoMapApiImplementation& api)
{
    m_pAppInputDelegate->BindApi(api);
}

void AppHost::UnbindApi()
{
    m_pAppInputDelegate->UnbindApi();
}

void AppHost::OnResume()
{
	m_pApp->OnResume();
}

void AppHost::OnPause()
{
	m_pApp->OnPause();
}

void AppHost::SetViewportOffset(float x, float y)
{
}

void AppHost::NotifyScreenPropertiesChanged(const Eegeo::Rendering::ScreenProperties& screenProperties)
{
    m_pApp->NotifyScreenPropertiesChanged(screenProperties);
}

void AppHost::Update(float dt)
{
    Eegeo::Modules::Map::MapModule& mapModule = m_pWorld->GetMapModule();
    if (!mapModule.IsRunning() && m_pAppLocationDelegate->HasReceivedPermissionResponse())
    {
        Eegeo::Space::LatLong ll( 0.0f, 0.0f );
        m_piOSLocationService->GetLocation( ll );
        
        Eegeo::Space::LatLongAltitude lla( ll.GetLatitude(), ll.GetLongitude(), 100.0f );
        Eegeo::Space::EcefTangentBasis cameraInterestBasis;
        Eegeo::Camera::CameraHelpers::EcefTangentBasisFromPointAndHeading(lla.ToECEF(), 0.0f, cameraInterestBasis);
        m_pApp->GetGlobeCameraController().SetInterestBasis( cameraInterestBasis );
        mapModule.Start();
    }
	m_pApp->Update(dt);
}

void AppHost::Draw(float dt)
{
	m_pApp->Draw(dt);
}


