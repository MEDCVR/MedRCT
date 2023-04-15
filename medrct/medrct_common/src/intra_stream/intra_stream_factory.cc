#include <medrct/intra_stream/intra_stream_factory.hh>
#include <medrct/intra_stream/intra_stream.hh>
#include "helper.hh"

namespace medrct
{
namespace stream
{

Stream::Ptr IntraStreamFactory::create(const YAML::Node& config) const
{
  std::string input_or_output_type;
  std::string topic_name;
  std::string name;
  std::string data_type;
  if (YAML::Node n = config["type"])
    input_or_output_type = n.as<std::string>();
  else
    throw std::runtime_error("No [type] key in controller config");
  if (YAML::Node n = config["topic_name"])
    topic_name = n.as<std::string>();
  else
    throw std::runtime_error("No [topic_name] key in controller config");
  if (YAML::Node n = config["name"])
    name = n.as<std::string>();
  else
    throw std::runtime_error("No [name] key in controller config");
  if (YAML::Node n = config["data_type"])
    data_type = n.as<std::string>();
  else
    throw std::runtime_error("No [data_type] key in controller config");

  Stream::Ptr stream;
  if (input_or_output_type == "input")
  {
    stream =
        createStreamWithDataType<IntraSubStream>(topic_name, name, data_type);
  }
  else if (input_or_output_type == "output")
  {
    stream =
        createStreamWithDataType<IntraPubStream>(topic_name, name, data_type);
  }
  else
    throw std::runtime_error(
        "Intra stream type can only be 'input', or 'output'");
  return stream;
}
} // namespace stream
} // namespace medrct
