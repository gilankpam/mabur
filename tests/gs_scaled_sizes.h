#pragma once
// THE bake set for the shipped GS OSD atlas -- one definition, shared by the
// synthetic-font tests (test_gs_overlay.cpp) and the real-asset gate
// (test_gs_asset.cpp), so the two can never drift apart.
//
// The union of the eight design sizes (19,21,22,24,26,34,38,56) multiplied by
// the four scales that give exact hits at 720p/1080p/1440p/2160p (x2/3, x1,
// x4/3, x2), each rounded with the SAME "(int)(px*scale+0.5)" rule layout()
// itself uses, deduplicated:
//
//   720p (x2/3):  13,14,15,16,17,23,25,37
//   1080p (x1):   19,21,22,24,26,34,38,56
//   1440p (x4/3): 25,28,29,32,35,45,51,75
//   2160p (x2):   38,42,44,48,52,68,76,112
//
// Every one of layout()'s 7 named sizes resolves EXACTLY (d=0, no snapping at
// all) at every one of the four resolutions with this list -- asserted by
// sizes_resolve_exactly_at_every_resolution (synthetic) and by
// asset_sizes_resolve_exactly_at_every_resolution (the real asset).
//
// Two prior attempts at deriving this union by hand were wrong in different
// ways (a flat +-2px guess, then a "roughly" enumeration missing 23/37/68/75
// and carrying a stray 72) -- both happened to still pass every test because
// pick()'s 15% tolerance quietly bridged the gaps, which is exactly the
// failure mode this comment and the exact-match tests exist to rule out.
//
// This string is ALSO what generated the shipped asset: it is the --sizes
// argument in gen_gsfont.py's "regenerating the shipped asset" recipe, and
// test_gs_asset.cpp asserts the committed .gfont bakes exactly these sizes
// and no others. Change one, change all three.
constexpr const char* kScaledSizes =
    "13,14,15,16,17,19,21,22,23,24,25,26,28,29,32,34,35,37,38,42,44,45,48,51,"
    "52,56,68,75,76,112";
