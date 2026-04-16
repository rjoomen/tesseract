# Tesseract Core Migration Guide: String-to-ID API Changes

Instructions for Claude Code to migrate downstream Tesseract projects
(tesseract_planning, trajopt, tesseract_ros, etc.) to the refactored
tesseract core on `feature/integer-link-ids`.

## Background

The core tesseract project replaced string-based identifiers with type-safe
`LinkId` / `JointId` objects. These are `NameId<Tag>` wrappers around a
`uint64_t` hash, with an implicit constructor from `std::string` / `const char*`.
Because the constructor is implicit, many call sites compile without changes.
The breaks come from removed types, renamed members, changed container key
types, and changed return types.

---

## Step 1: Build and collect compiler errors

```bash
cd /path/to/build
cmake --build . -j$(nproc) 2>&1 | tee build_errors.txt
```

The compiler errors ARE the migration checklist. Match each error to the
fix table below. Fix in batches by error type, rebuild, repeat until clean.

---

## Step 2: Fix compiler errors using this lookup table

### `fromName is not a member of NameId`

The static factory was removed. The constructor does the same thing.

```
SEARCH:  LinkId::fromName(   or   JointId::fromName(
FIX:     Remove ::fromName( wrapper, keep the argument.
         LinkId::fromName("x")  →  LinkId("x")   or just   "x"
```

### `ObjectPairKey was not declared`

The string-pair type alias was removed from the collision namespace.

```
SEARCH:  ObjectPairKey
FIX:     Replace type with tesseract::common::LinkIdPair.
         Construction: std::make_pair(a, b)  →  LinkIdPair(a, b)
NOTE:    LinkIdPair canonically orders elements (first().value() <= second().value()),
         so LinkIdPair(a, b) == LinkIdPair(b, a). This differs from std::pair.
```

### `make is not a member of LinkIdPair` / `OrderedIdPair`

The static factory was replaced by the constructor.

```
SEARCH:  LinkIdPair::make(
FIX:     LinkIdPair::make(a, b)  →  LinkIdPair(a, b)
```

### `invalid use of member function 'first' (did you forget the '()'?)`

`LinkIdPair` members are now private with const accessor methods.

```
SEARCH:  .first  and  .second  on LinkIdPair variables (NOT on std::pair or map iterators)
FIX:     .first  →  .first()       .second  →  .second()
```

### `has no member named 'link_names'` (on ContactResult)

The member was renamed to use IDs.

```
SEARCH:  \.link_names\b  on ContactResult variables
FIX:     .link_names  →  .link_ids
         If the string value is needed:  result.link_ids[0].name()
```

### `has no member named 'joint_names'` (on JointState)

The member was renamed to use IDs.

```
SEARCH:  \.joint_names\b  on JointState variables
FIX:     .joint_names  →  .joint_ids
         To get strings:  tesseract::common::toNames(state.joint_ids)
         Or:              state.getJointNames()
```

### `has no member named 'base_link_name'` / `joint_names` / `link_names` (on KDLTreeData)

All KDLTreeData members were renamed from strings to IDs.

```
SEARCH:  .base_link_name   →  .base_link_id
         .joint_names      →  .joint_ids
         .active_joint_names → .active_joint_ids
         .floating_joint_names → .floating_joint_ids
         .link_names        →  .link_ids
         .active_link_names →  .active_link_ids
         .static_link_names →  .static_link_ids
NOTE:    .floating_joint_values type changed from TransformMap to JointIdTransformMap
```

### `has no member named 'segment_index'` type mismatch (on KDLChainData)

KDLChainData members changed from string-keyed to ID-keyed.

```
segment_index:  map<string, int>              →  map<LinkId, int>
chains:         vector<pair<string, string>>  →  vector<pair<LinkId, LinkId>>
```

### `no matching function for call to 'getCollisionObjectPairs'`

Parameters changed from string vectors to ID vectors.

```
FIX:     Convert inputs:
         auto active_ids = tesseract::common::toIds<LinkId>(active_link_names);
         auto static_ids = tesseract::common::toIds<LinkId>(static_link_names);
         auto pairs = getCollisionObjectPairs(active_ids, static_ids, validator);
```

### `no matching function for call to 'parseSceneGraph'`

Parameters changed from strings to IDs.

```
BEFORE:  parseSceneGraph(sg, vector<string>, unordered_map<string,double>, TransformMap)
AFTER:   parseSceneGraph(sg, vector<JointId>, unordered_map<JointId,double>, JointIdTransformMap)
FIX:     Convert inputs with toIds<JointId>() and rebuild maps with JointId keys.
```

### `cannot convert 'const LinkId' to 'const string&'` (on getRoot())

The return type changed.

```
SEARCH:  .getRoot()  where result is assigned to std::string
FIX:     scene_graph.getRoot()  →  scene_graph.getRoot().name()  (if string needed)
         Or change the receiving variable to  const LinkId&
```

### `TransformMap was not declared` / `TransformMap is not a member`

The string-keyed TransformMap type alias was removed.

```
SEARCH:  \bTransformMap\b
FIX:     Determine whether keys are joints or links:
         Joint transforms  →  JointIdTransformMap
         Link transforms   →  LinkIdTransformMap
NOTE:    CalibrationInfo::joints changed from TransformMap to JointIdTransformMap.
         Iteration:  cal.first (was string)  →  cal.first.name() (now JointId)
```

### `no matching function` on EnvironmentMonitorInterface subclass

New pure virtual methods were added. String overloads are now delegating defaults.

```
FIX:     Implement the new ID-keyed pure virtuals in your subclass:

         bool setEnvironmentState(const std::string& ns,
             const SceneState::JointValues& joints,
             const JointIdTransformMap& floating_joints = {}) const override;

         bool setEnvironmentState(const std::string& ns,
             const std::vector<JointId>& joint_ids,
             const Eigen::Ref<const Eigen::VectorXd>& joint_values,
             const JointIdTransformMap& floating_joints = {}) const override;

         Same for the all-namespaces variants (returning vector<string>).

NOTE:    Existing string-based overrides still work but are no longer required.
         You can remove them and inherit the delegating defaults from the base class.
```

### `no matching function` / type mismatch on StateSolver or MutableStateSolver subclass

ID-keyed methods are now the pure virtuals; string methods are delegating defaults.

```
FIX:     1. Implement the ID-based pure virtual methods (not the string ones).
         2. Add using declarations to prevent name hiding:
              using StateSolver::setState;
              using StateSolver::getState;
              using StateSolver::getLinkTransforms;
              using StateSolver::getJacobian;
         3. String-based methods in MutableStateSolver were removed entirely —
            only ID-based pure virtuals remain.
```

---

## Step 3: Grep-based sweep for issues the compiler won't catch

These are semantic changes where the code still compiles but behaves differently
or accesses the wrong type. Run each grep, inspect matches, fix as needed.

### 3a. Map iteration extracting string keys

Old ID-keyed maps (`SceneState::joints`, `link_transforms`, etc.) used string
keys. Code that iterates them and uses `.first` as a string will compile
(structured bindings just change the type) but string operations like
concatenation (`+`) or stream insertion (`<<`) won't work on `JointId`/`LinkId`.

```bash
# Find iteration over SceneState fields
grep -rn "scene_state\.\(joints\|link_transforms\|joint_transforms\|floating_joints\)" --include="*.cpp" --include="*.h"
# For each match that iterates (for/range-based), check if .first is used as a string.
# FIX: id.name() to get the string
```

### 3b. ContactResultMap iteration

Keys changed from `pair<string,string>` to `LinkIdPair` with method accessors.

```bash
grep -rn "ContactResultMap\|contact_results\|contact_map" --include="*.cpp" --include="*.h"
# For iteration matches, check for .first/.second on the key.
# FIX: key.first  →  key.first()   and   key.second  →  key.second()
# FIX: key.first().name()  if string needed
```

### 3c. ChainGroup construction

Type changed from `vector<pair<string,string>>` to `vector<pair<LinkId,LinkId>>`.
Implicit conversion handles `make_pair(LinkId, LinkId)` but not
`make_pair(string, string)` assigned to the new pair type.

```bash
grep -rn "ChainGroup\|chain_group" --include="*.cpp" --include="*.h"
# Check construction sites. Brace initialization { {"base", "tip"} } may need
# explicit LinkId: { {LinkId("base"), LinkId("tip")} }
```

### 3d. ManipulatorInfo tcp_offset

The variant changed from `variant<string, Isometry3d>` to `variant<LinkId, Isometry3d>`.
Code using `std::get<std::string>()` will fail at compile time.

```bash
grep -rn "tcp_offset" --include="*.cpp" --include="*.h"
# FIX: std::get<std::string>(info.tcp_offset)  →  std::get<LinkId>(info.tcp_offset).name()
#      info.tcp_offset.index() == 0 still works (LinkId is index 0)
```

---

## Step 4: Build and test

```bash
cmake --build . -j$(nproc)
ctest --output-on-failure -j$(nproc)
```

If new errors appear, go back to Step 2.

---

## Reference: Convenience helpers

```cpp
#include <tesseract/common/types.h>

// Convert vector<string> → vector<IdT>
auto link_ids = tesseract::common::toIds<LinkId>(link_name_strings);
auto joint_ids = tesseract::common::toIds<JointId>(joint_name_strings);

// Convert any container of NameId<Tag> → vector<string>
auto names = tesseract::common::toNames(link_ids);
auto names = tesseract::common::toNames(joint_ids);
```

---

## Reference: What compiles transparently (no changes needed)

Because `NameId` has an implicit constructor from `std::string` and `const char*`,
single-value call sites compile without changes:

```cpp
LinkId id = "my_link";                    // implicit conversion
some_function_taking_LinkId("my_link");   // implicit at call site
JointId jid = joint_name_string;          // implicit from std::string variable
```

This covers most calls to `SceneGraph`, `AllowedCollisionMatrix`,
`CollisionMarginData`, `Environment`, `Link`/`Joint` constructors, and
collision manager methods. If a single-string parameter compiled before,
it almost certainly still compiles.

Implicit conversion does NOT help when:
- You need to construct **containers** (`vector<LinkId>` from `vector<string>`) — use `toIds()`
- You iterate **map keys** that changed type — use `.name()` on the ID
- You use **removed types** (`ObjectPairKey`, `TransformMap`) — see Step 2
- A **return type** changed (`getRoot()` returns `LinkId` not `string`)

---

## Reference: Full type mapping

| Old | New |
|---|---|
| `LinkId::fromName("x")` | `LinkId("x")` or `"x"` |
| `JointId::fromName("x")` | `JointId("x")` or `"x"` |
| `ObjectPairKey` | `tesseract::common::LinkIdPair` |
| `LinkIdPair::make(a, b)` | `LinkIdPair(a, b)` |
| `.first` / `.second` (LinkIdPair) | `.first()` / `.second()` |
| `TransformMap` | `LinkIdTransformMap` / `JointIdTransformMap` |
| `CalibrationInfo::joints` | type: `JointIdTransformMap` |
| `ContactResult::link_names` | `ContactResult::link_ids` |
| `JointState::joint_names` | `JointState::joint_ids` |
| `KDLTreeData::base_link_name` | `KDLTreeData::base_link_id` |
| `KDLTreeData::joint_names` | `KDLTreeData::joint_ids` |
| `KDLTreeData::active_joint_names` | `KDLTreeData::active_joint_ids` |
| `KDLTreeData::floating_joint_names` | `KDLTreeData::floating_joint_ids` |
| `KDLTreeData::link_names` | `KDLTreeData::link_ids` |
| `KDLTreeData::active_link_names` | `KDLTreeData::active_link_ids` |
| `KDLTreeData::static_link_names` | `KDLTreeData::static_link_ids` |
| `KDLChainData::segment_index` | key: `LinkId` (was `string`) |
| `KDLChainData::chains` | `vector<pair<LinkId,LinkId>>` |
| `ChainGroup` | `vector<pair<LinkId,LinkId>>` |
| `SceneState::joints` | key: `JointId` (was `string`) |
| `SceneState::link_transforms` | `LinkIdTransformMap` |
| `SceneState::joint_transforms` | `JointIdTransformMap` |
| `SceneState::floating_joints` | `JointIdTransformMap` |
| `SceneGraph::getRoot()` | returns `LinkId` (was `string`) |
| `ManipulatorInfo::tcp_offset` | `variant<LinkId, Isometry3d>` (was `variant<string, Isometry3d>`) |
