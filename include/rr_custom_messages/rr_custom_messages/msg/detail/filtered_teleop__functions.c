// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rr_custom_messages:msg/FilteredTeleop.idl
// generated code does not contain a copyright notice
#include "rr_custom_messages/msg/detail/filtered_teleop__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rr_custom_messages__msg__FilteredTeleop__init(rr_custom_messages__msg__FilteredTeleop * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // v
  // w
  return true;
}

void
rr_custom_messages__msg__FilteredTeleop__fini(rr_custom_messages__msg__FilteredTeleop * msg)
{
  if (!msg) {
    return;
  }
  // id
  // v
  // w
}

bool
rr_custom_messages__msg__FilteredTeleop__are_equal(const rr_custom_messages__msg__FilteredTeleop * lhs, const rr_custom_messages__msg__FilteredTeleop * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // v
  if (lhs->v != rhs->v) {
    return false;
  }
  // w
  if (lhs->w != rhs->w) {
    return false;
  }
  return true;
}

bool
rr_custom_messages__msg__FilteredTeleop__copy(
  const rr_custom_messages__msg__FilteredTeleop * input,
  rr_custom_messages__msg__FilteredTeleop * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // v
  output->v = input->v;
  // w
  output->w = input->w;
  return true;
}

rr_custom_messages__msg__FilteredTeleop *
rr_custom_messages__msg__FilteredTeleop__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rr_custom_messages__msg__FilteredTeleop * msg = (rr_custom_messages__msg__FilteredTeleop *)allocator.allocate(sizeof(rr_custom_messages__msg__FilteredTeleop), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rr_custom_messages__msg__FilteredTeleop));
  bool success = rr_custom_messages__msg__FilteredTeleop__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rr_custom_messages__msg__FilteredTeleop__destroy(rr_custom_messages__msg__FilteredTeleop * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rr_custom_messages__msg__FilteredTeleop__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rr_custom_messages__msg__FilteredTeleop__Sequence__init(rr_custom_messages__msg__FilteredTeleop__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rr_custom_messages__msg__FilteredTeleop * data = NULL;

  if (size) {
    data = (rr_custom_messages__msg__FilteredTeleop *)allocator.zero_allocate(size, sizeof(rr_custom_messages__msg__FilteredTeleop), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rr_custom_messages__msg__FilteredTeleop__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rr_custom_messages__msg__FilteredTeleop__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rr_custom_messages__msg__FilteredTeleop__Sequence__fini(rr_custom_messages__msg__FilteredTeleop__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rr_custom_messages__msg__FilteredTeleop__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rr_custom_messages__msg__FilteredTeleop__Sequence *
rr_custom_messages__msg__FilteredTeleop__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rr_custom_messages__msg__FilteredTeleop__Sequence * array = (rr_custom_messages__msg__FilteredTeleop__Sequence *)allocator.allocate(sizeof(rr_custom_messages__msg__FilteredTeleop__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rr_custom_messages__msg__FilteredTeleop__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rr_custom_messages__msg__FilteredTeleop__Sequence__destroy(rr_custom_messages__msg__FilteredTeleop__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rr_custom_messages__msg__FilteredTeleop__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rr_custom_messages__msg__FilteredTeleop__Sequence__are_equal(const rr_custom_messages__msg__FilteredTeleop__Sequence * lhs, const rr_custom_messages__msg__FilteredTeleop__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rr_custom_messages__msg__FilteredTeleop__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rr_custom_messages__msg__FilteredTeleop__Sequence__copy(
  const rr_custom_messages__msg__FilteredTeleop__Sequence * input,
  rr_custom_messages__msg__FilteredTeleop__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rr_custom_messages__msg__FilteredTeleop);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rr_custom_messages__msg__FilteredTeleop * data =
      (rr_custom_messages__msg__FilteredTeleop *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rr_custom_messages__msg__FilteredTeleop__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rr_custom_messages__msg__FilteredTeleop__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rr_custom_messages__msg__FilteredTeleop__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
