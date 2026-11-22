#include "model_cache.hpp"
#include <algorithm>
#include <regex>
#include <set>

namespace harness::observation {
namespace {
const std::set<std::string> names{"manifest.json","vision.onnx","sensor.onnx","embodiment.onnx","fusion.onnx","decision.onnx"};
bool sha(const std::string &s) { return std::regex_match(s,std::regex("[0-9a-f]{64}")); }
Json identity(const Json &r) {
  return {{"profile_id",r.at("profile_id")},{"binding_hash",r.at("binding_hash")},
          {"active_proposition_ids",r.at("active_proposition_ids")},{"mode",r.at("mode")},
          {"runtime_context",r.value("runtime_context",Json(nullptr))},
          {"enforcement_authorized",false}};
}
}
ModelCache::ModelCache(fs::path root):root_(std::move(root)) {
  fs::create_directories(root_);
  fs::permissions(root_,fs::perms::owner_all,fs::perm_options::replace);
  // A previous process's receipt is not evidence of a loaded engine here.
}
Json ModelCache::refresh(const Json &r,const Download &download,const Admit &admit) {
  if (r.value("status","")!="available") {
    const auto status=r.value("status","");
    if (status=="inactive" || status=="unassigned" || status=="binding_changed") {
      active_=nullptr;atomic_json(root_/"active.json",nullptr);
    }
    atomic_json(root_/"status.json",{{"status",status},{"active",active_}});
    return {{"status",status},{"active",active_}};
  }
  require(r.value("mode","")=="shadow" && !r.value("enforcement_authorized",true),"unsupported model authority");
  require(std::regex_match(r.at("model_id").get<std::string>(),std::regex("physical-[0-9a-f]{64}")),"invalid model identity");
  const auto desired=identity(r);
  if (active_.is_object() && identity(active_)!=desired) {
    active_=nullptr;atomic_json(root_/"active.json",nullptr);
  }
  const auto hash=r.at("download").at("sha256").get<std::string>();
  require(sha(hash),"invalid bundle hash");
  const auto directory=root_/"objects"/hash;
  fs::create_directories(directory);
  const auto &files=r.at("files");
  require(files.is_array() && files.size()==names.size(),"six exact model files required");
  std::set<std::string> seen;
  std::uint64_t total=0;
  for (const auto &file:files) {
    const auto name=file.at("name").get<std::string>();
    require(names.contains(name) && seen.insert(name).second,"invalid model filename");
    require(sha(file.at("sha256").get<std::string>()),"invalid model file hash");
    const auto size=file.at("size").get<std::int64_t>();
    require(size>0 && size<=512*1024*1024 && total+static_cast<std::uint64_t>(size)<=512*1024*1024,"model size bound exceeded");
    total+=static_cast<std::uint64_t>(size);
    const auto target=directory/name;
    auto valid=[&](const fs::path &p) {return fs::is_regular_file(p) && !fs::is_symlink(p) &&
      fs::file_size(p)==static_cast<std::uint64_t>(size) && file_hash(p)==file.at("sha256").get<std::string>();};
    if (valid(target)) continue;
    const auto temporary=directory/(name+".partial");
    fs::remove(temporary);
    try {
      download(file,temporary);
      require(valid(temporary),"downloaded model identity mismatch");
      fs::rename(temporary,target);
    } catch (...) {fs::remove(temporary);throw;}
  }
  const auto manifest=read_json(directory/"manifest.json");
  require(manifest.value("trained",false) && manifest.value("tensor_contract_version",0)==2 &&
    manifest.at("registry_hash")==r.at("metadata").at("registry_hash") &&
    manifest.at("architecture")==r.at("metadata").at("architecture"),"model manifest differs from resolution");
  std::set<std::string> supported;
  for (const auto &p:manifest.at("propositions")) supported.insert(p.at("id"));
  std::set<std::string> selected;
  for (const auto &p:r.at("active_proposition_ids"))
    require(supported.contains(p.get<std::string>()) && selected.insert(p.get<std::string>()).second,"unsupported active proposition");
  require(!selected.empty(),"no active propositions");
  for (const auto &[name,graph]:manifest.at("graphs").items()) {
    require(names.contains(graph.at("file").get<std::string>()) && name+".onnx"==graph.at("file").get<std::string>(),"invalid graph path");
    require(file_hash(directory/graph.at("file").get<std::string>())==graph.at("sha256").get<std::string>(),"graph differs from manifest");
  }
  require(manifest.at("graphs").size()==5,"five physical graphs required");
  Json receipt=desired;
  receipt["metadata"]=r.at("metadata");receipt["files"]=Json::array();
  for(const auto &file:files)receipt["files"].push_back({{"name",file.at("name")},{"size",file.at("size")},{"sha256",file.at("sha256")}});
  receipt.update({{"model_id",r.at("model_id")},{"scope",r.at("scope")},
    {"bundle_sha256",hash},{"path",directory.string()},{"running",false}});
  atomic_json(root_/"prepared.json",receipt);
  if (active_.is_object() && active_.at("model_id")==receipt.at("model_id"))
    return {{"status","unchanged"},{"active",active_}};
  if (!admit(directory,r)) {
    atomic_json(root_/"status.json",{{"status","pending_runtime"},{"prepared",receipt},{"active",active_}});
    return {{"status","pending_runtime"},{"prepared",receipt},{"active",active_}};
  }
  active_=receipt;active_["running"]=false;active_["admitted"]=true;
  atomic_json(root_/"active.json",active_);
  atomic_json(root_/"status.json",{{"status","loaded"},{"active",active_}});
  return {{"status","loaded"},{"active",active_}};
}
}
