// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rr_custom_messages:msg/FilteredTeleop.idl
// generated code does not contain a copyright notice

#include "rr_custom_messages/msg/detail/filtered_teleop__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
const rosidl_type_hash_t *
rr_custom_messages__msg__FilteredTeleop__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf6, 0x9e, 0xf9, 0x7c, 0x85, 0x57, 0x8b, 0xd7,
      0xcd, 0xad, 0xb1, 0xbd, 0xe5, 0x59, 0x03, 0x40,
      0x6b, 0x7f, 0xa8, 0x14, 0x7a, 0x80, 0xb5, 0x9a,
      0x35, 0x86, 0xc8, 0x9c, 0xee, 0x93, 0x75, 0xb6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rr_custom_messages__msg__FilteredTeleop__TYPE_NAME[] = "rr_custom_messages/msg/FilteredTeleop";

// Define type names, field names, and default values
static char rr_custom_messages__msg__FilteredTeleop__FIELD_NAME__id[] = "id";
static char rr_custom_messages__msg__FilteredTeleop__FIELD_NAME__v[] = "v";
static char rr_custom_messages__msg__FilteredTeleop__FIELD_NAME__w[] = "w";

static rosidl_runtime_c__type_description__Field rr_custom_messages__msg__FilteredTeleop__FIELDS[] = {
  {
    {rr_custom_messages__msg__FilteredTeleop__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rr_custom_messages__msg__FilteredTeleop__FIELD_NAME__v, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rr_custom_messages__msg__FilteredTeleop__FIELD_NAME__w, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rr_custom_messages__msg__FilteredTeleop__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rr_custom_messages__msg__FilteredTeleop__TYPE_NAME, 37, 37},
      {rr_custom_messages__msg__FilteredTeleop__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 id\n"
  "float32 v\n"
  "float32 w";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rr_custom_messages__msg__FilteredTeleop__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rr_custom_messages__msg__FilteredTeleop__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 28, 28},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rr_custom_messages__msg__FilteredTeleop__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rr_custom_messages__msg__FilteredTeleop__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
