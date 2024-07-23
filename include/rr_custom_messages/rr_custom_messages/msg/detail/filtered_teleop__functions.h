// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rr_custom_messages:msg/FilteredTeleop.idl
// generated code does not contain a copyright notice

#ifndef RR_CUSTOM_MESSAGES__MSG__DETAIL__FILTERED_TELEOP__FUNCTIONS_H_
#define RR_CUSTOM_MESSAGES__MSG__DETAIL__FILTERED_TELEOP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "rr_custom_messages/msg/rosidl_generator_c__visibility_control.h"

#include "rr_custom_messages/msg/detail/filtered_teleop__struct.h"

/// Initialize msg/FilteredTeleop message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rr_custom_messages__msg__FilteredTeleop
 * )) before or use
 * rr_custom_messages__msg__FilteredTeleop__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
bool
rr_custom_messages__msg__FilteredTeleop__init(rr_custom_messages__msg__FilteredTeleop * msg);

/// Finalize msg/FilteredTeleop message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
void
rr_custom_messages__msg__FilteredTeleop__fini(rr_custom_messages__msg__FilteredTeleop * msg);

/// Create msg/FilteredTeleop message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rr_custom_messages__msg__FilteredTeleop__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
rr_custom_messages__msg__FilteredTeleop *
rr_custom_messages__msg__FilteredTeleop__create();

/// Destroy msg/FilteredTeleop message.
/**
 * It calls
 * rr_custom_messages__msg__FilteredTeleop__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
void
rr_custom_messages__msg__FilteredTeleop__destroy(rr_custom_messages__msg__FilteredTeleop * msg);

/// Check for msg/FilteredTeleop message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
bool
rr_custom_messages__msg__FilteredTeleop__are_equal(const rr_custom_messages__msg__FilteredTeleop * lhs, const rr_custom_messages__msg__FilteredTeleop * rhs);

/// Copy a msg/FilteredTeleop message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
bool
rr_custom_messages__msg__FilteredTeleop__copy(
  const rr_custom_messages__msg__FilteredTeleop * input,
  rr_custom_messages__msg__FilteredTeleop * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
const rosidl_type_hash_t *
rr_custom_messages__msg__FilteredTeleop__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
const rosidl_runtime_c__type_description__TypeDescription *
rr_custom_messages__msg__FilteredTeleop__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
const rosidl_runtime_c__type_description__TypeSource *
rr_custom_messages__msg__FilteredTeleop__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
const rosidl_runtime_c__type_description__TypeSource__Sequence *
rr_custom_messages__msg__FilteredTeleop__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/FilteredTeleop messages.
/**
 * It allocates the memory for the number of elements and calls
 * rr_custom_messages__msg__FilteredTeleop__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
bool
rr_custom_messages__msg__FilteredTeleop__Sequence__init(rr_custom_messages__msg__FilteredTeleop__Sequence * array, size_t size);

/// Finalize array of msg/FilteredTeleop messages.
/**
 * It calls
 * rr_custom_messages__msg__FilteredTeleop__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
void
rr_custom_messages__msg__FilteredTeleop__Sequence__fini(rr_custom_messages__msg__FilteredTeleop__Sequence * array);

/// Create array of msg/FilteredTeleop messages.
/**
 * It allocates the memory for the array and calls
 * rr_custom_messages__msg__FilteredTeleop__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
rr_custom_messages__msg__FilteredTeleop__Sequence *
rr_custom_messages__msg__FilteredTeleop__Sequence__create(size_t size);

/// Destroy array of msg/FilteredTeleop messages.
/**
 * It calls
 * rr_custom_messages__msg__FilteredTeleop__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
void
rr_custom_messages__msg__FilteredTeleop__Sequence__destroy(rr_custom_messages__msg__FilteredTeleop__Sequence * array);

/// Check for msg/FilteredTeleop message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
bool
rr_custom_messages__msg__FilteredTeleop__Sequence__are_equal(const rr_custom_messages__msg__FilteredTeleop__Sequence * lhs, const rr_custom_messages__msg__FilteredTeleop__Sequence * rhs);

/// Copy an array of msg/FilteredTeleop messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rr_custom_messages
bool
rr_custom_messages__msg__FilteredTeleop__Sequence__copy(
  const rr_custom_messages__msg__FilteredTeleop__Sequence * input,
  rr_custom_messages__msg__FilteredTeleop__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RR_CUSTOM_MESSAGES__MSG__DETAIL__FILTERED_TELEOP__FUNCTIONS_H_
