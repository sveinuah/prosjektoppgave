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
    camera_front_right_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontRightCamera")))->GetChildActor());
    camera_front_left_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontLeftCamera")))->GetChildActor());
    camera_front_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontCenterCamera")))->GetChildActor());
    camera_back_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BackCenterCamera")))->GetChildActor());
    camera_bottom_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BottomCenterCamera")))->GetChildActor());
	camera_right_front_ = Cast<APIPCamera>(
		(UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("RightFrontCamera")))->GetChildActor());
	camera_left_front_ = Cast<APIPCamera>(
		(UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("LeftFrontCamera")))->GetChildActor());
	camera_right_back_ = Cast<APIPCamera>(
		(UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("RightBackCamera")))->GetChildActor());
	camera_left_back_ = Cast<APIPCamera>(
		(UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("LeftBackCamera")))->GetChildActor());
}

void AFlyingPawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    pawn_events_.getPawnTickSignal().emit(DeltaSeconds);
}


void AFlyingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_right_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_center_ = nullptr;
    camera_back_center_ = nullptr;
    camera_bottom_center_ = nullptr;
	camera_right_front_ = nullptr;
	camera_left_front_ = nullptr;
	camera_left_back_ = nullptr;
	camera_right_back_ = nullptr;

    Super::EndPlay(EndPlayReason);
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> AFlyingPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("bottom_center", camera_bottom_center_);
    cameras.insert_or_assign("back_center", camera_back_center_);
	cameras.insert_or_assign("right_front", camera_right_front_);
	cameras.insert_or_assign("left_front", camera_left_front_);
	cameras.insert_or_assign("right_back", camera_right_back_);
	cameras.insert_or_assign("left_back", camera_left_back_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_bottom_center_);
    cameras.insert_or_assign("4", camera_back_center_);
	cameras.insert_or_assign("5", camera_right_front_);
	cameras.insert_or_assign("6", camera_left_front_);
	cameras.insert_or_assign("7", camera_right_back_);
	cameras.insert_or_assign("8", camera_left_back_);

    cameras.insert_or_assign("", camera_front_center_);
    cameras.insert_or_assign("fpv", camera_front_center_);

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

