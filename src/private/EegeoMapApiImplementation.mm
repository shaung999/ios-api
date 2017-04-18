// Copyright eeGeo Ltd (2012-2014), All Rights Reserved

#import "EegeoMapApiImplementation.h"
#import "EGPolygonImplementation.h"
#import "EGPrecacheOperationImplementation.h"
#import "EGMapDelegate.h"

#include "CameraHelpers.h"
#include "EcefTangentBasis.h"
#include "PinsModule.h"
#include "ITextureFileLoader.h"
#include "RegularTexturePageLayout.h"
#include "CameraTransitioner.h"
#include "PrecacheOperationScheduler.h"
#include "AnnotationController.h"
#include "ICityThemesService.h"
#include "ICityThemesUpdater.h"
#include "EnvironmentFlatteningService.h"
#include "ICityThemeRepository.h"
#include "GlobeCameraControllerConfiguration.h"
#include "CityThemesModule.h"

#include "RouteBuilder.h"
#include "VectorMath.h"
#include "RouteStyle.h"
#include "RouteService.h"
#include "Route.h"
#include "IdentityRouteThicknessPolicy.h"
#include "MathsHelpers.h"
#include "AggregateCollisionBvhProvider.h"

@implementation EegeoMapApiImplementation
{
    Eegeo::Routes::Style::Thickness::IdentityRouteThicknessPolicy m_routeThicknessPolicy;
    Eegeo::EegeoWorld* m_pWorld;
    ExampleApp* m_pApp;
    id<EGMapDelegate> m_delegate;

    Eegeo::Api::CameraTransitioner* m_pCameraTransitioner;
    Eegeo::Api::PrecacheOperationScheduler m_precacheOperationScheduler;
    Eegeo::Api::AnnotationController* m_pAnnotationController;
    
    Eegeo::Selection::Building* m_pBuildingSelection;
    Eegeo::Collision::RayCasterResult m_rayCasterResult;
}

@synthesize selectedAnnotations = _selectedAnnotations;

- (id)initWithWorld:(Eegeo::EegeoWorld&)world
                app:(ExampleApp&)app
           delegate:(id<EGMapDelegate>)delegate
               view:(UIView*)view
{
    m_pWorld = &world;
    m_pApp = &app;
    m_delegate = delegate;

    m_pCameraTransitioner = Eegeo_NEW(Eegeo::Api::CameraTransitioner)(m_pApp->GetGlobeCameraController());
    
    Eegeo::Modules::Map::MapModule& mapModule = m_pWorld->GetMapModule();

    //    Eegeo::Collision::AggregateCollisionBvhProvider& acbp = mapModule.GetAggregateCollisionBvhProvider();
    Eegeo::Collision::EnvironmentRayCaster* pRayCaster = new Eegeo::Collision::EnvironmentRayCaster( mapModule.GetAggregateCollisionBvhProvider(), mapModule.GetEnvironmentFlatteningService() );
    
    m_pBuildingSelection = Eegeo_NEW(Eegeo::Selection::Building)( pRayCaster );
    
    Eegeo::Modules::IPlatformAbstractionModule& platformAbstractionModule = m_pWorld->GetPlatformAbstractionModule();
    Eegeo::Modules::Core::RenderingModule& renderingModule = m_pWorld->GetRenderingModule();
    Eegeo::Modules::Map::Layers::TerrainModelModule& terrainModelModule = m_pWorld->GetTerrainModelModule();
    const Eegeo::Rendering::ScreenProperties& initialScreenProperties = m_pApp->GetScreenPropertiesProvider().GetScreenProperties();
    Eegeo::Helpers::ITextureFileLoader& textureFileLoader = platformAbstractionModule.GetTextureFileLoader();
    Eegeo::Resources::Terrain::Heights::TerrainHeightProvider& terrainHeightProvider = m_pWorld->GetMapModule().GetTerrainModelModule().GetTerrainHeightProvider();
    
    m_pAnnotationController = Eegeo_NEW(Eegeo::Api::AnnotationController)(renderingModule,
                                                                          platformAbstractionModule,
                                                                          terrainModelModule,
                                                                          mapModule,
                                                                          initialScreenProperties,
                                                                          textureFileLoader,
                                                                          terrainHeightProvider,
                                                                          view,
                                                                          delegate);
        
    return self;
}

- (void)teardown
{
    Eegeo_DELETE(m_pCameraTransitioner);
    Eegeo_DELETE(m_pAnnotationController);
    Eegeo_DELETE(m_pBuildingSelection);
}

- (void)update:(float)dt
{
    static double previousAlt = 0.0f;
    
    if(m_pCameraTransitioner->IsTransitioning())
    {
        m_pCameraTransitioner->Update(dt);
    }
    
    Eegeo::Camera::RenderCamera renderCamera(m_pApp->GetGlobeCameraController().GetRenderCamera());

    currentAltitude = renderCamera.GetAltitude();
    if ( previousAlt != currentAltitude )
    {
        if ( [m_delegate respondsToSelector:@selector(altitudeChangedTo:distanceToInterest:)] )
        {
            dispatch_async(dispatch_get_main_queue(), ^{
                
                [m_delegate altitudeChangedTo:currentAltitude distanceToInterest:m_pApp->GetGlobeCameraController().GetDistanceToInterest()];
            });
        }
        previousAlt = currentAltitude;
    }
    
    m_precacheOperationScheduler.Update();
    m_pAnnotationController->Update(dt, renderCamera);
    m_pApp->GetBuildingFootprintsModule().Update( dt );
}

- (CGFloat)getCurrentAltitude
{
    Eegeo::Camera::RenderCamera renderCamera(m_pApp->GetGlobeCameraController().GetRenderCamera());
    return renderCamera.GetAltitude();
}

- (CGFloat)getCurrentDistanceToInterest
{
    return m_pApp->GetGlobeCameraController().GetDistanceToInterest();
}

- (void)updateScreenProperties:(const Eegeo::Rendering::ScreenProperties&)screenProperties
{
    m_pAnnotationController->UpdateScreenProperties(screenProperties);
}

- (id<EGPolygon>)polygonWithCoordinates:(CLLocationCoordinate2D *)coords
                                 count:(NSUInteger)count
{
    std::vector<Eegeo::Space::LatLongAltitude> verts;
    verts.reserve(count);

    for(int i = 0; i < count; i++)
    {
        const CLLocationCoordinate2D coord = coords[i];
        verts.push_back(Eegeo::Space::LatLongAltitude::FromDegrees(coord.latitude, coord.longitude, 0.0));
    }

    auto* pModel = Eegeo::Data::Geofencing::GeofenceModel::GeofenceBuilder(
            "test",
            Eegeo::v4(1.0f, 0.0f, 0.0f, 0.5f),
            verts)
            .Build();

    EGPolygonImplementation* wrapper = [[[EGPolygonImplementation alloc] initWithGeofence:*pModel] autorelease];

    return wrapper;
}

- (void)addPolygon:(id<EGPolygon>)polygon
{
    if([polygon isKindOfClass:[EGPolygonImplementation class]])
    {
        EGPolygonImplementation* pImpl = (EGPolygonImplementation *)polygon;

        Eegeo::Data::Geofencing::GeofenceController& geofenceController =
                m_pWorld->GetDataModule().GetGeofenceModule().GetController();

        geofenceController.Add(*pImpl.pGeofenceModel);
    }
}

- (void)removePolygon:(id <EGPolygon>)polygon
{
    if([polygon isKindOfClass:[EGPolygonImplementation class]])
    {
        EGPolygonImplementation* pImpl = (EGPolygonImplementation *)polygon;

        Eegeo::Data::Geofencing::GeofenceController& geofenceController =
                m_pWorld->GetDataModule().GetGeofenceModule().GetController();

        geofenceController.Remove(*pImpl.pGeofenceModel);
    }
}

- (void)setCenterCoordinate:(CLLocationCoordinate2D)centerCoordinate
             distanceMetres:(float)distanceMetres
         orientationDegrees:(float)orientationDegrees
                   animated:(BOOL)animated
{
    Eegeo::Space::LatLongAltitude location = Eegeo::Space::LatLongAltitude::FromDegrees(centerCoordinate.latitude,
                                                                                        centerCoordinate.longitude,
                                                                                        distanceMetres);
    
    if(!animated)
    {
        Eegeo::Space::EcefTangentBasis cameraInterestBasis;
        Eegeo::Camera::CameraHelpers::EcefTangentBasisFromPointAndHeading(location.ToECEF(),
                                                                          orientationDegrees,
                                                                          cameraInterestBasis);
        m_pApp->SetCameraView(cameraInterestBasis, distanceMetres);
    }
    else
    {
        if ( orientationDegrees >= 0.0f )
            m_pCameraTransitioner->StartTransitionTo(location.ToECEF(), distanceMetres, orientationDegrees, true);
        else
            m_pCameraTransitioner->StartTransitionTo(location.ToECEF(), distanceMetres, true);
    }
}

- (CLLocationCoordinate2D)getCenterCoordinate
{
    const Eegeo::dv3& interestPointEcef = m_pApp->GetGlobeCameraController().GetEcefInterestPoint();
    
    Eegeo::Space::LatLongAltitude interestPoint = Eegeo::Space::LatLongAltitude::FromECEF(interestPointEcef);
    
    CLLocationCoordinate2D centerCoord = CLLocationCoordinate2DMake(interestPoint.GetLatitudeInDegrees(), interestPoint.GetLongitudeInDegrees());
    
    return centerCoord;
}

- (void)addAnnotation:(id<EGAnnotation>)annotation
{
    m_pAnnotationController->InsertAnnotation(annotation);
}

- (void)selectAnnotation:(id<EGAnnotation>)annotation animated:(BOOL)animated
{
    m_pAnnotationController->SelectAnnotation(annotation, animated);
}

- (void)deselectAnnotation:(id<EGAnnotation>)annotation animated:(BOOL)animated
{
    m_pAnnotationController->DeselectAnnotation(annotation, animated);
}

- (NSArray*)selectedAnnotations
{
    id<EGAnnotation> selectedAnnotation = m_pAnnotationController->SelectedAnnotation();
    return (selectedAnnotation ? @[ selectedAnnotation ] : @[]);
}

- (void)removeAnnotation:(id<EGAnnotation>)annotation
{
    m_pAnnotationController->RemoveAnnotation(annotation);
    m_pAnnotationController->Update(0.f, m_pApp->GetGlobeCameraController().GetRenderCamera());
}

- (EGAnnotationView*)viewForAnnotation:(id<EGAnnotation>)annotation
{
    return m_pAnnotationController->ViewForAnnotation(annotation);
}

- (void)zoomToAltitude:(float)altitude
{
    Eegeo::dv3 centre = m_pApp->GetGlobeCameraController().GetEcefInterestPoint();
    m_pCameraTransitioner->StartTransitionTo(centre, altitude, true);
}

- (void)setVisibleCoordinateBounds:(EGCoordinateBounds)bounds animated:(BOOL)animated
{
    Eegeo::dv3 ne = Eegeo::Space::LatLong::FromDegrees(bounds.ne.latitude, bounds.ne.longitude).ToECEF();
    Eegeo::dv3 sw = Eegeo::Space::LatLong::FromDegrees(bounds.sw.latitude, bounds.sw.longitude).ToECEF();
    Eegeo::dv3 center = (ne + sw) / 2.0;
    double boundingRadius = (ne - sw).Length() / 2.0;
    
    const float altitude = [self computeAltitudeForRadius:boundingRadius];
    
    if(animated)
    {
        m_pCameraTransitioner->StartTransitionTo(center, altitude, 0.f, 45.f, true);
    }
    else
    {
        const float currentBearingRads = Eegeo::Camera::CameraHelpers::GetAbsoluteBearingRadians(center,
                                                                                                 m_pApp->GetGlobeCameraController().GetInterestBasis().GetForward());
        
        Eegeo::Space::EcefTangentBasis cameraInterestBasis;
        Eegeo::Camera::CameraHelpers::EcefTangentBasisFromPointAndHeading(center,
                                                                          Eegeo::Math::Rad2Deg(currentBearingRads),
                                                                          cameraInterestBasis);
        
        m_pApp->SetCameraView(cameraInterestBasis, altitude, 45.f);
    }
}


- (id<EGPrecacheOperation>)precacheMapDataInCoordinateBounds:(EGCoordinateBounds)bounds
{
    Eegeo::dv3 ne = Eegeo::Space::LatLong::FromDegrees(bounds.ne.latitude, bounds.ne.longitude).ToECEF();
    Eegeo::dv3 sw = Eegeo::Space::LatLong::FromDegrees(bounds.sw.latitude, bounds.sw.longitude).ToECEF();
    Eegeo::dv3 ecefCenter = (ne + sw) / 2.0;
    double boundingRadius = fmax((ecefCenter - ne).Length(), (ecefCenter - sw).Length());

    
    EGPrecacheOperationImplementation* pEGPrecacheOperation = [[[EGPrecacheOperationImplementation alloc]
                                                                initWithPrecacheService:m_pWorld->GetStreamingModule().GetPrecachingService()
                                                                scheduler:m_precacheOperationScheduler
                                                                center:ecefCenter
                                                                radius:boundingRadius
                                                                delegate:m_delegate] autorelease];
    
    m_precacheOperationScheduler.EnqueuePrecacheOperation(*pEGPrecacheOperation);
    return pEGPrecacheOperation;
}

- (void)setMapTheme:(EGMapTheme*)mapTheme
{
    Eegeo::Resources::CityThemes::ICityThemesService& cityThemesService = m_pWorld->GetCityThemesModule().GetCityThemesService();
    Eegeo::Resources::CityThemes::ICityThemesUpdater& cityThemesUpdater = m_pWorld->GetCityThemesModule().GetCityThemesUpdater();

    if(mapTheme.enableThemeByLocation)
    {
        // bit shonky: semantic meaning of themeName changes based on whether it is an explicitly
        // named theme, or a location-based name of a season. Underlying C++ API needs some love.
        cityThemesUpdater.SetEnabled(true);
        cityThemesUpdater.SetThemeMustContain([mapTheme.themeName UTF8String]);
        cityThemesService.RequestTransitionToState([mapTheme.themeStateName UTF8String], 1.0f);
    }
    else
    {
        Eegeo::Resources::CityThemes::ICityThemeRepository& cityThemesRepository = m_pWorld->GetCityThemesModule().GetCityThemesRepository();
        const Eegeo::Resources::CityThemes::CityThemeData* pTheme = cityThemesRepository.GetThemeDataByName([mapTheme.themeName UTF8String]);

        if(pTheme != NULL)
        {
            cityThemesUpdater.SetEnabled(false);
            cityThemesService.SetSpecificTheme(*pTheme);
            std::string state = [mapTheme.themeStateName UTF8String];
            cityThemesService.RequestTransitionToState(state, 1.0f);
        }
    }
}

- (void)setEnvironmentFlatten:(BOOL)flatten
{
    Eegeo::Rendering::EnvironmentFlatteningService& flatteningService = m_pWorld->GetMapModule().GetEnvironmentFlatteningService();
    flatteningService.SetIsFlattened(flatten);
}

- (float)computeAltitudeForRadius:(double)boundingRadius
{
    // This ain't pretty.
    
    float tentativeAltitudeMetres = 0.f;
    float tentativeFovDeg = 60.0f;
    const double minimumAltitude = 500.0;
    const size_t NumIterations = 20;
    
    for(size_t i = 0; i < NumIterations; ++ i)
    {
        const float tentativeFovRad = Eegeo::Math::Deg2Rad(tentativeFovDeg);
        tentativeAltitudeMetres = static_cast<float>(fmax(minimumAltitude, (boundingRadius / tanf(tentativeFovRad/2.f))));
        
        Eegeo::Camera::GlobeCamera::GlobeCameraControllerConfiguration camConfig = Eegeo::Camera::GlobeCamera::GlobeCameraControllerConfiguration::CreateDefault(false);
        if(tentativeAltitudeMetres >= camConfig.globeModeBeginFOVChangeAltitude)
        {
            float fovZoomParam = Eegeo::Math::Clamp01((tentativeAltitudeMetres - camConfig.globeModeBeginFOVChangeAltitude) / (camConfig.globeModeEndFOVChangeAltitude - camConfig.globeModeBeginFOVChangeAltitude));
            tentativeFovDeg = Eegeo::Math::Lerp(camConfig.fovZoomedInGlobe, camConfig.fovZoomedOutGlobe, fovZoomParam);
        }
        else
        {
            float fovZoomParam = Eegeo::Math::Clamp01((tentativeAltitudeMetres - camConfig.zoomAltitudeLow) / (camConfig.globeModeBeginFOVChangeAltitude  - camConfig.zoomAltitudeLow));
            tentativeFovDeg = Eegeo::Math::Lerp(camConfig.fovZoomedInCity, camConfig.fovZoomedInGlobe, fovZoomParam);
        }
    }
    
    return tentativeAltitudeMetres;
}

- (CLLocationCoordinate2D)getCameraLatLong
{
    Eegeo::dv3 camera = m_pApp->GetGlobeCameraController().GetEcefInterestPoint();
    Eegeo::Space::LatLong latlong = Eegeo::Space::LatLong::FromECEF( camera );
    return CLLocationCoordinate2DMake(latlong.GetLatitudeInDegrees(), latlong.GetLongitudeInDegrees());
}

- (void)disableTraffic:(BOOL)disable
{
    m_pWorld->GetTrafficModule().SetEnabled( disable );
}

- (void)createRoute:(NSArray*)plotPoints Altitude:(float)altitudeMeters Red:(float)red Green:(float)green Blue:(float)blue Alpha:(float)alpha
{
    const float halfWidth = 5.f;
    const float routeSpeedMetersPerSecond = 40.f;
    
    //The color format is (Red, Green, Blue, Transparency - 0.0 is fully transparent and 1.0 is fully opaque).
    Eegeo::v4 routeColour(red, green, blue, alpha);
    
    //The route builder helper object provides a fluent interface to make building a route simpler.
    Eegeo::Routes::RouteBuilder builder;
    
    //We want the set of route vertices, and we get it by starting with an initial color and width and
    //adding points. This route starts near the Transamerica Pyramid.
    //
    //We can add points by (latDegrees, longDegrees, altitudeMeters) tuples or by a LatLongAltitide object..
    //
    //The color can be changed arbitrarily along the route.
    //
    builder.Start(routeColour, halfWidth, routeSpeedMetersPerSecond, Eegeo::Routes::Road);

    for ( int i = 0; i < plotPoints.count; i++ )
    {
        NSDictionary* dict = plotPoints[i];
        builder.AddPoint( [dict[@"latitude"] doubleValue], [dict[@"longitude"] doubleValue], altitudeMeters );
    }
    std::vector<Eegeo::Routes::RouteVertex> points = builder.FinishRoute();
    
    // A route thickness scaling policy should be provided; this informs the route how it should modify its thickness
    // (for example, based on camera altitude, or to play a "pulse" animation). Two implementations are provided; the
    // IdentityRouteThicknessPolicy and the LinearAltitudeScaleBasedRouteThicknessPolicy. For this example we use the
    // identity policy which will not modify the thickness of the route. The style accepts a const reference, so it
    // does not take ownership over the thickness policy.
    Eegeo::Routes::Style::RouteStyle routeStyle( &m_routeThicknessPolicy, Eegeo::Routes::Style::RouteStyle::DebugStyleNone);
    
    //We can now create a route from this set of points.
    //
    //The route can be created using the CreateRoute method on the RouteService, which must
    //be destroyed through the complementary DestroyRoute method on RouteService.
    Eegeo::Routes::Route* route = m_pWorld->GetRoutesModule().GetRouteService().CreateRoute(points, routeStyle, false);
}

- (void)buildingHighlight:(BOOL)select latitude:(double)latitude longitude:(double)longitude
{
    
}

- (BOOL)Event_TouchRotate:(const AppInterface::RotateData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchRotate_Start:(const AppInterface::RotateData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchRotate_End:(const AppInterface::RotateData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchPinch:(const AppInterface::PinchData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchPinch_Start:(const AppInterface::PinchData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchPinch_End:(const AppInterface::PinchData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchPan:(const AppInterface::PanData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchPan_Start:(const AppInterface::PanData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchPan_End:(const AppInterface::PanData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchTap:(const AppInterface::TapData&)data
{
    Eegeo::v2 screenTapPoint = Eegeo::v2(data.point.GetX(), data.point.GetY());
    if ( !m_pAnnotationController->HandleTap(screenTapPoint) )
    {
        // ** Only check for building tap if delegate is implemented and not an annotation controller **
        if ( [m_delegate respondsToSelector:@selector(buildingSelectedLatitude:longitude:)] )
        {
            // ** Tap pressed, lets see if building selection required **
            const Eegeo::Camera::RenderCamera& renderCamera = m_pApp->GetGlobeCameraController().GetRenderCamera();
            
            float screenPixelX = data.point.GetX();
            float screenPixelY = data.point.GetY();
            
            Eegeo::dv3 rayOrigin = renderCamera.GetEcefLocation();
            Eegeo::dv3 rayDirection;
            Eegeo::Camera::CameraHelpers::GetScreenPickRay(renderCamera, screenPixelX, screenPixelY, rayDirection);
            
            Eegeo::Space::LatLongAltitude lla( .0f, .0f, .0f );
            if ( m_pBuildingSelection->PerformRayPick(rayOrigin, rayDirection, lla) )
            {
                m_rayCasterResult = m_pBuildingSelection->GetRayCasterResult();
                
                m_pApp->GetBuildingFootprintsModule().GetBuildingSelectionController().PerformOperation( m_rayCasterResult, Eegeo::BuildingFootprints::BuildingSelectionController::Select );
                
                // ** Let the delegate know **
                dispatch_async( dispatch_get_main_queue(),^{
                    
                    [m_delegate buildingSelectedLatitude:lla.GetLatitudeInDegrees() longitude:lla.GetLongitudeInDegrees()];
                    });
            }
        }
    }
    
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (void)switchBuildingHighlightOff
{
    m_pApp->GetBuildingFootprintsModule().GetBuildingSelectionController().PerformOperation( m_rayCasterResult, Eegeo::BuildingFootprints::BuildingSelectionController::Deselect );
}

- (BOOL)Event_TouchDoubleTap:(const AppInterface::TapData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    if ( [m_delegate respondsToSelector:@selector(doubleTap:)] )
        [m_delegate doubleTap:data];
    
    return eventConsumed;
}

- (BOOL)Event_TouchDown:(const AppInterface::TouchData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchMove:(const AppInterface::TouchData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

- (BOOL)Event_TouchUp:(const AppInterface::TouchData&)data
{
    bool eventConsumed = m_pCameraTransitioner->IsTransitioning();
    return eventConsumed;
}

@end

namespace Eegeo
{
    namespace Selection
    {
        BOOL Building::PerformRayPick(dv3& rayOrigin, const dv3& rayDirection, Space::LatLongAltitude& intersectionPointLLA)
        {
            const Collision::RayCasterResult& pickResult = m_pRayCaster->CastRay(rayOrigin, rayDirection, Collision::CollisionGroup::Buildings);
            if (pickResult.intersects)
            {
                m_pickResult = pickResult;
                intersectionPointLLA = Space::LatLongAltitude::FromECEF( pickResult.intersectionPointEcef );
                return YES;
            }
            
            return NO;
        }
    }
}
