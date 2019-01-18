#include "FlyingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"

AFlyingPawn::AFlyingPawn()
{
    pawn_events_.getActuatorSignal().connect_member(this, &AFlyingPawn::setRotorSpeed);
}

void AFlyingPawn::BeginPlay()
{
    Super::BeginPlay();

    for (auto i = 0; i < rotor_count; ++i) {
        rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
    }
}

void AFlyingPawn::initializeForBeginPlay()
{    
    //get references of existing camera
    camera_forward_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontCenterCamera")))->GetChildActor());
    camera_backward_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BackCenterCamera")))->GetChildActor());
    camera_down_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BottomCenterCamera")))->GetChildActor());
	camera_right_center_ = Cast<APIPCamera>(
		(UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("RightCenterCamera")))->GetChildActor());
	camera_left_center_ = Cast<APIPCamera>(
		(UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("LeftCenterCamera")))->GetChildActor());
}

void AFlyingPawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    pawn_events_.getPawnTickSignal().emit(DeltaSeconds);
}


void AFlyingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_forward_center_ = nullptr;
    camera_backward_center_ = nullptr;
    camera_down_center_ = nullptr;
	camera_right_center_ = nullptr;
	camera_left_center_ = nullptr;

    Super::EndPlay(EndPlayReason);
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> AFlyingPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("forward_center", camera_forward_center__);
    cameras.insert_or_assign("down_center", camera_down_center_);
    cameras.insert_or_assign("backward_center", camera_backward_center_);
	cameras.insert_or_assign("right_center", camera_right_center_);
	cameras.insert_or_assign("left_center", camera_left_center_);

    cameras.insert_or_assign("0", camera_forward_center_);
    cameras.insert_or_assign("1", camera_down_center_);
    cameras.insert_or_assign("2", camera_backward_center_);
	cameras.insert_or_assign("3", camera_right_center_);
	cameras.insert_or_assign("4", camera_left_center_);

    cameras.insert_or_assign("", camera_forward_center_);
    cameras.insert_or_assign("fpv", camera_forward_center_);

    return cameras;
}

void AFlyingPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

void AFlyingPawn::setRotorSpeed(const std::vector<MultirotorPawnEvents::RotorInfo>& rotor_infos)
{
    for (auto rotor_index = 0; rotor_index < rotor_infos.size(); ++rotor_index) {
        auto comp = rotating_movements_[rotor_index];
        if (comp != nullptr) {
            comp->RotationRate.Yaw = 
                rotor_infos.at(rotor_index).rotor_speed * rotor_infos.at(rotor_index).rotor_direction *
                180.0f / M_PIf * RotatorFactor;
        }
    }
}

