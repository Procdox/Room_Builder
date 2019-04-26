#pragma once
#include "Grid_Region.h"

typedef FLL<Region<Pgrd> *> Region_List;

//==========================================================================================================
//========================================== transforms ====================================================
//==========================================================================================================
FVector2D convert(Pgrd const &target);

TArray<FVector2D> convert(FLL<Pgrd> const &target);

TArray<FVector2D> toFVector(FLL<Pgrd> const &target);

void mergeGroup(Region_List & nulls);

///<summary>
///<para>Trims small sub-regions from target with Cull and merges them to outs</para>
///<para>&#160;</para>
///<para>Assumes: -</para>
///<para>Fulfills: outs is maximally merged. target is maximally merged</para>
///</summary>
void removeSmallSections(Region_List &target, grd const &min_width, Region_List &smalls);

///<summary>
///<para>Trims small sub-regions from target with Cull and merges them to outs</para>
///<para>&#160;</para>
///<para>Assumes: target is maximally merged</para>
///<para>Fulfills: outs is maximally merged. target is simple, target are otherwise restricted to width</para>
///</summary>
void sizeRestrict(Region_List &target, grd const &min_width, Region_List &outs);

///<summary>
///<para>Allocates a novel set of sub-regions, from a set of regions, as defined by a boundary.</para>
///<para>&#160;</para>
///<para>Assumes: -</para>
///<para>Fulfills: set is maximally merged. result is maximally merged</para>
///</summary>
Region_List allocateBoundaryFrom(FLL<Pgrd> const &boundary, Region_List &set);

///<summary>
///<para>Allocates a novel set of sub-regions, from a set of regions, as defined by a boundary.</para>
///<para>&#160;</para>
///<para>Assumes: -</para>
///<para>Fulfills: set is maximally merged. result is maximally merged</para>
///</summary>
void allocateBoundaryFromInto(FLL<Pgrd> const &boundary, Region_List &set, Region_List &result);

///<summary>
///<para>Allocates a novel set of sub-regions, from a set of regions, as defined by a boundary. Then trims small sub-regions with Cull and returns them to the originating set of regions.</para>
///<para>&#160;</para>
///<para>Assumes: -</para>
///<para>Fulfills: set is maximally merged. result is simple, merges are otherwise restricted to width</para>
///</summary>
Region_List allocateCleanedBoundaryFrom(FLL<Pgrd> const &boundary, grd const &min_width, Region_List &set);

///<summary>
///<para>Allocates a novel set of sub-regions, from a set of regions, as defined by a boundary. Then trims small sub-regions with Cull and returns them to the originating set of regions.</para>
///<para>&#160;</para>
///<para>Assumes: -</para>
///<para>Fulfills: set is maximally merged. result is simple, merges are otherwise restricted to width</para>
///</summary>
void allocateCleanedBoundaryFromInto(FLL<Pgrd> const &boundary, grd const &min_width, Region_List &set, Region_List &result);