// Copyright eeGeo Ltd (2012-2014), All Rights Reserved

#pragma once

#import <UIKit/UIKit.h>
#import "EGMapApi.h"
#import "EGApi.h"
#include "EegeoWorld.h"
#include "ExampleApp.h"
#include "AppInterface.h"
#include "EnvironmentRayCaster.h"
#include "LatLongAltitude.h"
#include "BuildingFootprintsModule.h"
#include "BuildingFootprints.h"
#include "BuildingSelectionController.h"
#include "CityThemeData.h"

@interface EegeoMapApiImplementation : NSObject<EGMapApi>
{
    CGFloat currentAltitude;
}

- (id)initWithWorld:(Eegeo::EegeoWorld&)world
                app:(ExampleApp&)app
           delegate:(id<EGMapDelegate>)delegate
               view:(UIView*)view;

- (void)teardown;

- (void)update:(float)dt;

- (void)updateScreenProperties:(const Eegeo::Rendering::ScreenProperties&)screenProperties;

- (void)disableTraffic:(BOOL)disable;

- (void)createRoute:(NSArray*)plotPoints Altitude:(float)altitudeMeters Red:(float)red Green:(float)green Blue:(float)blue Alpha:(float)alpha;
- (void)selectBuildingAtX:(double)X Y:(double)Y Z:(double)Z;

- (CLLocationCoordinate2D)getCameraLatLong;
- (void)zoomToAltitude:(float)altitude centreLat:(double)centreLat centreLng:(double)centreLng;

- (BOOL)Event_TouchRotate:(const AppInterface::RotateData&)data;
- (BOOL)Event_TouchRotate_Start:(const AppInterface::RotateData&)data;
- (BOOL)Event_TouchRotate_End:(const AppInterface::RotateData&)data;

- (BOOL)Event_TouchPinch:(const AppInterface::PinchData&)data;
- (BOOL)Event_TouchPinch_Start:(const AppInterface::PinchData&)data;
- (BOOL)Event_TouchPinch_End:(const AppInterface::PinchData&)data;

- (BOOL)Event_TouchPan:(const AppInterface::PanData&)data;
- (BOOL)Event_TouchPan_Start:(const AppInterface::PanData&)data;
- (BOOL)Event_TouchPan_End:(const AppInterface::PanData&)data;

- (BOOL)Event_TouchTap:(const AppInterface::TapData&)data;
- (BOOL)Event_TouchDoubleTap:(const AppInterface::TapData&)data;

- (BOOL)Event_TouchDown:(const AppInterface::TouchData&)data;
- (BOOL)Event_TouchMove:(const AppInterface::TouchData&)data;
- (BOOL)Event_TouchUp:(const AppInterface::TouchData&)data;
- (CGFloat)getCurrentAltitude;

@end

namespace Eegeo
{
    namespace Selection
    {
        class Building
        {
        public:
            Building( Collision::EnvironmentRayCaster* pRayCaster ) : m_pRayCaster( pRayCaster ), m_pickResult() {}
            ~Building() { delete m_pRayCaster; }
            
            BOOL PerformRayPick(dv3& rayOrigin, const dv3& rayDirection, Space::LatLongAltitude& intersectionPointLLA);
            Collision::RayCasterResult GetRayCasterResult() { return m_pickResult; }
        private:
            Collision::EnvironmentRayCaster* m_pRayCaster;
            Collision::RayCasterResult m_pickResult;
        };
    }
}
