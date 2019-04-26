// Fill out your copyright notice in the Description page of Project Settings.

#include "room_description_builder.h"
#include "Grid_Tools.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"
#include "ConstructorHelpers.h"

#define JC_VORONOI_IMPLEMENTATION
#define JCV_REAL_TYPE double
#define JCV_ATAN2 atan2
#define JCV_FLT_MAX 1.7976931348623157E+308
#include "jc_voronoi.h"

//==========================================================================================================
//========================================= utilities ======================================================
//==========================================================================================================

#define color_Pnk FColor(200,0,200)
#define color_red FColor(255,0,0)
#define color_green FColor(0,255,0)
#define color_blue FColor(0,0,255)

void Draw_Border(const TArray<FVector2D> &border, float height, const UWorld *ref, FColor color = color_Pnk) {
	float offset = 0;
	for (int64 index = 0; index < border.Num(); index++) {
		int64 next = (index + 1) % border.Num();

		DrawDebugLine(
			ref,
			FVector(border[index], height + offset * 5),
			FVector(border[next], height + offset * 5),
			//FColor(FMath::RandRange(0,255), FMath::RandRange(0, 255), FMath::RandRange(0, 255)),
			color,
			true,
			-1,
			0,
			3
		);
		//offset++;
	}
}


Pgrd circularUniformPoint(grd radius = 1) {
	float t = 2 * PI * FMath::RandRange(0.f, 1.f);
	float u = FMath::RandRange(0.f, 1.f) + FMath::RandRange(0.f, 1.f);
	float r = u;
	if (u > 1)
		r = 2 - u;
	return Pgrd(r*cos(t),r*sin(t)) * radius;

}
Pgrd boxUniformPoint(grd width = 10, grd height = 10, int64 divisions = 100) {
	return Pgrd(width * (int64)FMath::RandRange(0, divisions), height * (int64)FMath::RandRange(0, divisions)) / divisions;
}
Pgrd boxUniformPoint(PBox const & box, int64 divisions = 100) {
	int64 X = (int64)FMath::RandRange(-(float)divisions, (float)divisions);
	int64 Y = (int64)FMath::RandRange(-(float)divisions, (float)divisions);

	return Pgrd(X, Y) * box.getExtent() + box.getCenter();
}

PBox getBounds(Region<Pgrd> * target) {

	PBox result;

	auto init = target->getBounds().last()->getRoot()->getStart()->getPosition();

	result.Max.X = init.X;
	result.Max.Y = init.Y;

	result.Min = result.Max;

	for(auto boundary : target->getBounds()){
		auto points = boundary->getLoopPoints();

		for (auto point : points) {
			result.Min.X = FMath::Min(result.Min.X, point.X);
			result.Min.Y = FMath::Min(result.Min.Y, point.Y);
			result.Max.X = FMath::Max(result.Max.X, point.X);
			result.Max.Y = FMath::Max(result.Max.Y, point.Y);
		}
	}

	return result;
}


void Aroom_description_builder::Create_System(Type_Tracker & tracker) {

	tracker.system->resetEdgeMarks();
	tracker.system->resetRegionMarks();

	for (auto ext : tracker.Exteriors) {
		cleanRegion(ext);
	}

	for (auto room : tracker.Rooms){
		room->mark = 1;
		cleanRegion(room);
	}

	for (auto hall : tracker.Halls) {
		hall->mark = 1;
		cleanRegion(hall);
	}

	for (auto ext : tracker.Exteriors) {
		//CreateWallSections(ext, 0, room_height, tracker);
	}

	for (auto room : tracker.Rooms) {
		Ainterior_builder* novel = (Ainterior_builder*)GetWorld()->SpawnActor(Ainterior_builder::StaticClass());
		Room_Layout layout(room, 0, room_height);
		novel->Create(&layout, &details);
	}

	for (auto hall : tracker.Halls) {
		Ainterior_builder* novel = (Ainterior_builder*)GetWorld()->SpawnActor(Ainterior_builder::StaticClass());
		Room_Layout layout(hall, 0, room_height);
		novel->Create(&layout, &details);
	}
}

//==========================================================================================================
//======================================= generation =======================================================
//==========================================================================================================

namespace shape_generators
{
	FLL<Pgrd> Square_Generator(grd x, grd y, Pgrd center) {
		FLL<Pgrd> boundary;
		boundary.append(Pgrd(-x, -y) + center);
		boundary.append(Pgrd(-x, y) + center);
		boundary.append(Pgrd(x, y) + center);
		boundary.append(Pgrd(x, -y) + center);

		return boundary;
	}
	FLL<Pgrd> Diamond_Generator(grd x, grd y, Pgrd center) {
		FLL<Pgrd> boundary;
		if (x < y) {
			grd d = y - x;
			boundary.append(Pgrd(-x, -d) + center);
			boundary.append(Pgrd(-x, d) + center);
			boundary.append(Pgrd(0, y) + center);
			boundary.append(Pgrd(x, d) + center);
			boundary.append(Pgrd(x, -d) + center);
			boundary.append(Pgrd(0, -y) + center);
		}
		else if (x > y) {
			grd d = x - y;
			boundary.append(Pgrd(-x, 0) + center);
			boundary.append(Pgrd(-d, y) + center);
			boundary.append(Pgrd(d, y) + center);
			boundary.append(Pgrd(x, 0) + center);
			boundary.append(Pgrd(d, -y) + center);
			boundary.append(Pgrd(-d, -y) + center);
		}
		else {
			boundary.append(Pgrd(-x, 0) + center);
			boundary.append(Pgrd(0, x) + center);
			boundary.append(Pgrd(x, 0) + center);
			boundary.append(Pgrd(0, -x) + center);
		}


		return boundary;
	}
	FLL<Pgrd> Bevel_Generator(grd x, grd y, Pgrd center) {
		grd level;
		if (x < y) {
			level = x / 4;
		}
		else {
			level = y / 4;
		}

		FLL<Pgrd> boundary;

		boundary.append(Pgrd(-x, level - y) + center);
		boundary.append(Pgrd(-x, y - level) + center);
		boundary.append(Pgrd(level - x, y) + center);
		boundary.append(Pgrd(x - level, y) + center);
		boundary.append(Pgrd(x, y - level) + center);
		boundary.append(Pgrd(x, level - y) + center);
		boundary.append(Pgrd(x - level, -y) + center);
		boundary.append(Pgrd(level - x, -y) + center);

		return boundary;
	}
	FLL<Pgrd> Rectangle_Generator(grd x, grd y, Pgrd center) {
		grd skew = (int64)FMath::RandRange((float)0, (float)10);
		Pgrd angle(skew / 10, (grd(10) - skew) / 10);
		Pgrd perp((skew - 10) / 10, skew / 10);

		FLL<Pgrd> boundary;

		boundary.append(center - angle * x - perp * y);
		boundary.append(center - angle * x + perp * y);
		boundary.append(center + angle * x + perp * y);
		boundary.append(center + angle * x - perp * y);

		return boundary;
	}

	typedef FLL<Pgrd>(*generatorFunc)(grd, grd, Pgrd);
}

FLL<Pgrd> Pick_Generator(grd x, grd y, Pgrd center) {
	using namespace shape_generators;

	generatorFunc generator = Square_Generator;
	float gen_choice = FMath::RandRange(0, 1);
	if (gen_choice > .4) {
		if (gen_choice > .7) {
			generator = Bevel_Generator;
			UE_LOG(LogTemp, Warning, TEXT("style: bevel\n"));
		}
		else {
			generator = Diamond_Generator;
			UE_LOG(LogTemp, Warning, TEXT("style: diamond\n"));
		}
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("style: square\n"));
	}

	return generator(x, y, center);
}



Region_List Type_Tracker::createRoom(Region_Suggestion const &suggested) {
	Region_List final_room_set;

	for (auto boundary : suggested.boundaries) {
		allocateBoundaryFromInto(*boundary, Nulls, final_room_set);
	}

	UE_LOG(LogTemp, Warning, TEXT("room smalls"));
	removeSmallSections(final_room_set, min_room_width, Nulls);

	UE_LOG(LogTemp, Warning, TEXT("hall smalls"));
	removeSmallSections(Nulls, min_hall_width, Smalls);

	Rooms.append(final_room_set);

	return final_room_set;
}

Region_List Type_Tracker::createHall(Region_Suggestion const &suggested) {
	Region_List final_hall_set;

	for (auto boundary : suggested.boundaries) {
		allocateBoundaryFromInto(*boundary, Nulls, final_hall_set);
	}

	removeSmallSections(final_hall_set, min_hall_width, Nulls);
	removeSmallSections(Nulls, min_hall_width, Smalls);

	Halls.append(final_hall_set);

	return final_hall_set;
}

Region_List Type_Tracker::createNull(Region_Suggestion const &suggested) {
	Region_List final_null_set;

	for (auto boundary : suggested.boundaries) {
		allocateBoundaryFromInto(*boundary, Exteriors, final_null_set);
	}

	removeSmallSections(final_null_set, min_room_width, Exteriors);

	Nulls.append(final_null_set);

	return final_null_set;
}

bool Region_Suggestion::contains(Pgrd const &test) {
	for (auto region : boundaries)
		if (getPointRelation(*region, test) != point_exterior)
			return true;
	return false;
}

void clusterSuggestions(FLL<Region_Suggestion*> &suggested, grd const &tolerance) {
	auto x = suggested.begin();
	for (auto x = suggested.begin(); x != suggested.end();++x) {
		for (auto y = x.next(); y != suggested.end();) {
			//try and find a pair of centroids within tolerance
			bool seperate = true;
			for (auto x_p : x->centroids) {
				for (auto y_p : y->centroids) {
					//if ((x_p - y_p).Size() < tolerance)
					if(x->contains(y_p) && y->contains(x_p)) {
						seperate = false;
						break;
					}
				}
				if (!seperate)
					break;
			}


			//if found, merge all of y into x
			if (seperate)
				++y;
			else {
				x->centroids.absorb(y->centroids);
				x->boundaries.absorb(y->boundaries);

				suggested.remove(*y);

				y = x.next();
			}
		}
	}
}

FLL<Region_Suggestion*> suggestDistribution(Pgrd const &A, Pgrd const &B, grd const &room_width, grd const &room_depth, grd const &min_hall_width, bool start_row = true, bool end_row = true) {
	FLL<Region_Suggestion*> result;

	Pgrd dir = B - A;
	dir.Normalize();
	Pgrd par(dir.Y, -dir.X);
	par *= (room_depth + min_hall_width / 2);

	grd full_segment = (B - A).Size() + (room_width * 2) - min_hall_width;
	int rooms = (full_segment / room_width).n;
	grd segment = full_segment / rooms;


	int i = 0;
	if (!start_row)
		i++;

	if (!end_row)
		rooms--;

	for (; i < rooms; i++) {
		grd offset = (segment * i) - room_width + min_hall_width / 2;
		Pgrd root = A + (dir * offset);

		{

			Region_Suggestion * suggest = new Region_Suggestion();

			FLL<Pgrd> * bounds = new FLL<Pgrd>();
			bounds->append(root);
			bounds->append(root + (dir * segment));
			bounds->append(root + par + (dir * segment));
			bounds->append(root + par);

			suggest->boundaries.append(bounds);
			suggest->centroids.append(root + (par / 2) + (dir * segment / 2));

			result.append(suggest);
		}

		{
			Region_Suggestion * suggest = new Region_Suggestion();

			FLL<Pgrd> * bounds = new FLL<Pgrd>();
			bounds->append(root - par);
			bounds->append(root - par + (dir * segment));
			bounds->append(root + (dir * segment));
			bounds->append(root);

			suggest->boundaries.append(bounds);
			suggest->centroids.append(root - (par / 2) + (dir * segment / 2));

			result.append(suggest);
		}
	}

	return result;
}

FLL<Pgrd> * wrapSegment(Pgrd const &A, Pgrd const &B, grd const &extent_perp, grd const &extent_ends) {
	FLL<Pgrd> * result = new FLL<Pgrd>();

	Pgrd dir = B - A;
	dir.Normalize();

	Pgrd par(-dir.Y, dir.X);

	dir *= extent_ends;
	par *= extent_perp;

	result->append(A - dir - par);
	result->append(A - dir + par);
	result->append(B + dir + par);
	result->append(B + dir - par);

	return result;
}

FLL<Pgrd> poissonSample(grd region_radius, int point_count, grd mid_seperation)
{
	//generate points
	FLL<Pgrd> point_list;

	int safety = 10 * point_count;
	while (point_list.size() < point_count && --safety > 0)
	{
		Pgrd suggestion = circularUniformPoint(region_radius);

		bool safe = true;
		for (auto point : point_list)
		{
			if ((point - suggestion).Size() < 10) {
				safe = false;
				break;
			}
		}
		if (safe)
			point_list.append(suggestion);
	}

	return point_list;
}

void Aroom_description_builder::buldingFromBlock(DCEL<Pgrd> * system, FLL<rigid_line> &list) 
{
	Region_List Exteriors;
	Region_List cells;
	Exteriors.append(system->region());

	const grd region(70);
	const int max_point_count = 100;
	const grd span(10);
	
	jcv_rect bounding_box = { { -region.n, -region.n }, { region.n, region.n } };

	jcv_point points[max_point_count];
	jcv_diagram diagram;

	FMemory::Memset<jcv_diagram>(diagram, 0);

	auto point_list = poissonSample(region, max_point_count, span);

	int i = 0;
	for (auto point : point_list)
	{
		points[i].x = point.X.n;
		points[i].y = point.Y.n;
		++i;
	}
	
	jcv_diagram_generate(point_list.size(), (const jcv_point *)points, &bounding_box, &diagram);

	point_list.clear();

	const jcv_site* sites = jcv_diagram_get_sites(&diagram);

	for (int i = 0; i < diagram.numsites; ++i) {
		Region_Suggestion cell_suggestion;
		FLL<Pgrd> cell_boundary;

		const jcv_site* site = &sites[i];

		const jcv_graphedge* e = site->edges;

		while(e)
		{
			cell_boundary.push(Pgrd(e->pos[0].x, e->pos[0].y));
			e = e->next;
		}

		Region_List novel_cells;
		allocateBoundaryFromInto(cell_boundary, Exteriors, novel_cells);
		cells.absorb(novel_cells);
		point_list.append(Pgrd(site->p.x, site->p.y));
	}

	
	Region_List central;
	Region_List peripheral;

	auto p = point_list.begin();
	for (auto cell : cells)
	{
		auto size = (*p).Size();
		UE_LOG(LogTemp, Warning, TEXT("size: %f"), size.n);
		if (size < 30)
		{
			central.append(cell);
		}
		else if (size < 50)
		{
			peripheral.append(cell);
		}
		++p;
	}

	mergeGroup(peripheral);

	for (auto cell : central)
	{
		Ainterior_builder* novel = (Ainterior_builder*)GetWorld()->SpawnActor(Ainterior_builder::StaticClass());
		Room_Layout layout(cell, 0, room_height);
		novel->Create(&layout, &details);
	}
	for (auto cell : peripheral)
	{
		Ainterior_builder* novel = (Ainterior_builder*)GetWorld()->SpawnActor(Ainterior_builder::StaticClass());
		Room_Layout layout(cell, 0, room_height);
		novel->Create(&layout, &details);
	}
}

//==========================================================================================================
//====================================== member specific ===================================================
//==========================================================================================================

void Aroom_description_builder::Main_Generation_Loop() {
	UE_LOG(LogTemp, Warning, TEXT("Main Generation"));
	
	if (use_static_seed)
		FMath::RandInit(random_seed);

	/*DCEL<Pgrd> system_new;
	Type_Tracker system_types(&system_new);

	

	UE_LOG(LogTemp, Warning, TEXT("SRand Seed %d\n"), random_seed);

	auto system_bounds = shape_generators::Square_Generator(100, 100, Pgrd(0, 0));

	system_types.Nulls.append(system_new.region(system_bounds));

	create_Layout(system_types, unalligned_count, alligned_count, GetWorld());*/

	DCEL<Pgrd> * system = new DCEL<Pgrd>();
	//Type_Tracker frame(system, min_room_width, min_hall_width);

	FLL<rigid_line> list;
	for (auto p : Lines)
		list.append(rigid_line(p));

	buldingFromBlock(system, list);

	//Create_System(frame);

	delete system;
}

Aroom_description_builder::Aroom_description_builder()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root = CreateDefaultSubobject<USceneComponent>(TEXT("GeneratedRoot"));

	RootComponent = root;

	use_static_seed = false;
	random_seed = 0;

	rooms_per_segment = 10;
	closets_per_segment = 6;

	room_height = 100;
	

	min_room_width = 18;
	min_hall_width = 12;

	room_width = 54;
	room_depth = 54;
	hall_width = 12;
}

// Called when the game starts or when spawned
void Aroom_description_builder::BeginPlay()
{
	Super::BeginPlay();

	//is the orientation of room creation correct?

	Main_Generation_Loop();

}

// Called every frame
void Aroom_description_builder::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

/*
	UE_LOG(LogTemp, Warning, TEXT("Building Generation\n\n"));
	Region_Suggestion null_suggestion;

	for (auto x : list) {

		FLL<Pgrd> * null_boundary = wrapSegment(x.start, x.end, room_depth + min_hall_width / 2, room_depth - min_hall_width / 2);

		null_suggestion.boundaries.append(null_boundary);
	}

	frame.createNull(null_suggestion);



	UE_LOG(LogTemp, Warning, TEXT("Hall Generation\n\n\n"));
	Region_Suggestion hall_suggestion;

	for (auto x : list) {

		FLL<Pgrd> * hall_boundary = wrapSegment(x.start, x.end, hall_width / 2, hall_width / 2);

		hall_suggestion.boundaries.append(hall_boundary);
	}

	frame.createHall(hall_suggestion);



	UE_LOG(LogTemp, Warning, TEXT("Stuff Generation\n\n\n"));

	FLL<Region_Suggestion *> room_list;

	for (auto x : list) {
		auto p = suggestDistribution(x.start, x.end, room_width, room_depth, min_hall_width, x.start_row, x.end_row);
		room_list.absorb(p);
	}

	for (auto room_suggestion : room_list) {
		for (auto p : room_suggestion->centroids)
			DrawDebugLine(
				GetWorld(),
				FVector(convert(p), 10),
				FVector(convert(p), 20),
				color_blue,
				true,
				-1,
				0,
				5
			);
	}

	clusterSuggestions(room_list, room_width);


	UE_LOG(LogTemp, Warning, TEXT("ROOMS\n"));
	for (auto room_suggestion : room_list) {
		FColor color(FMath::RandRange(0, 255), FMath::RandRange(0, 255), FMath::RandRange(0, 255));
		for (auto p : room_suggestion->centroids) {
			UE_LOG(LogTemp, Warning, TEXT("room at : %f, %f"), p.X.n, p.Y.n);
			DrawDebugLine(
				GetWorld(),
				FVector(convert(p), 20),
				FVector(convert(p), 30),
				color,
				true,
				-1,
				0,
				5
			);
		}

		//frame.createRoom(*room_suggestion);

		//for (auto p : room_suggestion->boundaries)
		//	Draw_Border(convert(*p), 30, GetWorld(), color);

		for(auto r : frame.createRoom(*room_suggestion))
			for (auto p : r->getBounds())
				Draw_Border(convert(p->getLoopPoints()), 50, GetWorld(), color);
	}
	UE_LOG(LogTemp, Warning, TEXT("SMALLS\n"));

	frame.Smalls.absorb(frame.Nulls);

	//display leftovers
	{
		for (auto n : frame.Smalls)
			for (auto p : n->getBounds())
				Draw_Border(convert(p->getLoopPoints()), 60, GetWorld(), FColor(0, 200, 0));
	}

	Region_List all_smalls;
	Region_List smalls;

	for (auto small : frame.Smalls) {

		//filters for neighboring rooms, removes them from frame consideration for potential edits
		Region_List neighbors = small->getNeighbors();
		Region_List room_neighbors;
		for (auto neighbor : neighbors)
			if (frame.Rooms.remove(neighbor))
				room_neighbors.append(neighbor);

		Region_List rooms;


		smalls.append(small);

		Region_List novel_smalls;
		Region_List relevant;

		for (auto potential : room_neighbors) {
			relevant.append(potential);

			for (auto part : smalls) {
				if (!merge(potential, part)) {
					novel_smalls.append(part);
				}
			}

			removeSmallSections(relevant, min_room_width, novel_smalls);

			smalls.clear();

			rooms.absorb(relevant);
			smalls.absorb(novel_smalls);
		}

		frame.Rooms.absorb(rooms);
		all_smalls.absorb(smalls);
	}

	frame.Smalls.clear();
	frame.Smalls.absorb(all_smalls);
*/