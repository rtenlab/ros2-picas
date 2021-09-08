@# Included from rosidl_generator_dds_idl/resource/idl.idl.em
@{
message = service.request_message
wrapper_prefix = 'Sample'
wrapper_type = wrapper_prefix + '_' + message.structure.namespaced_type.name
}@

@[for ns in message.structure.namespaced_type.namespaces]@
module @(ns) {

@[end for]@
module dds_ {

struct @(wrapper_type)_ {

unsigned long long client_guid_0_;
unsigned long long client_guid_1_;
long long sequence_number_;
@('::'.join(message.structure.namespaced_type.namespaces))::dds_::@(message.structure.namespaced_type.name)_ request_;

};

#pragma keylist @(wrapper_type)_

};  // module dds_

@[for ns in reversed(message.structure.namespaced_type.namespaces)]@
};  // module @(ns) {

@[end for]@
