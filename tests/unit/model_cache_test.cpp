#include "model_cache.hpp"
#include <fstream>
#include <iostream>
using namespace harness::observation;
int main() {
  const auto root=fs::temp_directory_path()/("critic-cache-test-"+unique_id());
  try {
    const auto source=root/"source";fs::create_directories(source);
    Json graphs=Json::object(),files=Json::array();
    for (const auto *name:{"vision","sensor","embodiment","fusion","decision"}) {
      const std::string file=std::string(name)+".onnx";
      std::ofstream(source/file)<<"graph bytes for "<<name;
      graphs[name]={{"file",file},{"sha256",file_hash(source/file)}};
    }
    const Json props=Json::array({{{"id","human.intrusion"},{"revision","1"}}});
    const Json metadata={{"architecture","fixture"},{"registry_hash",std::string(64,'a')},{"propositions",props}};
    Json manifest=metadata;manifest.update({{"trained",true},{"tensor_contract_version",2},{"graphs",graphs}});
    atomic_json(source/"manifest.json",manifest);
    for (const auto &entry:fs::directory_iterator(source)) files.push_back({{"name",entry.path().filename().string()},
      {"size",fs::file_size(entry.path())},{"sha256",file_hash(entry.path())}});
    Json offer={{"status","available"},{"model_id","physical-"+std::string(64,'a')},{"scope","generic"},
      {"profile_id","profile"},{"binding_hash","binding"},{"mode","shadow"},{"enforcement_authorized",false},
      {"active_proposition_ids",Json::array({"human.intrusion"})},{"metadata",metadata},
      {"download",{{"sha256",std::string(64,'a')}}},{"files",files}};
    ModelCache cache(root/"cache");int downloads=0,loads=0;
    auto download=[&](const Json &file,const fs::path &path){
      ++downloads;fs::copy_file(source/file.at("name").get<std::string>(),path);
    };
    auto load=[&](const fs::path &,const Json &){++loads;return true;};
    require(cache.refresh(offer,download,load).at("status")=="loaded","first load failed");
    require(downloads==6 && loads==1,"initial download count");
    const auto admitted=read_json(root/"cache"/"active.json");
    require(admitted.value("mode","")=="shadow" && !admitted.value("enforcement_authorized",true),
            "persisted admission cannot enter the shadow-only physical runtime");
    require(cache.refresh(offer,download,load).at("status")=="unchanged","same model changed");
    require(downloads==6 && loads==1,"unchanged model downloaded or loaded again");
    auto newer=offer;newer["model_id"]="physical-"+std::string(64,'b');newer["scope"]="client";
    newer["download"]["sha256"]=std::string(64,'b');
    require(cache.refresh(newer,download,[](const fs::path &,const Json &){return false;}).at("status")=="pending_runtime","failed loader promoted");
    require(cache.active().at("model_id")==offer.at("model_id"),"failed update replaced working model");
    const auto count=downloads;
    require(cache.refresh(newer,download,load).at("status")=="loaded","retry failed");
    require(downloads==count && cache.active().at("scope")=="client","retry redownloaded cached files");
    auto bad=newer;bad["files"][0]["name"]="../escape";
    bool rejected=false;try {cache.refresh(bad,download,load);} catch(...) {rejected=true;}
    require(rejected && !fs::exists(root/"escape"),"unsafe filename admitted");
    bad=newer;bad["download"]["sha256"]=std::string(64,'c');
    rejected=false;try {cache.refresh(bad,[](const Json &,const fs::path &p){std::ofstream(p)<<"corrupt";},load);} catch(...) {rejected=true;}
    require(rejected && cache.active().at("model_id")==newer.at("model_id"),"corrupt update replaced active model");
    cache.refresh({{"status","inactive"}},download,load);
    require(cache.active().is_null(),"deactivation retained active model");
    ModelCache restarted(root/"cache");require(restarted.active().is_null(),"restart claimed old engine loaded");
    restarted.refresh(newer,download,load);require(downloads==count,"restart redownloaded valid cached files");
    fs::remove_all(root);std::cout<<"model cache checks passed\n";return 0;
  } catch(const std::exception &error) {fs::remove_all(root);std::cerr<<error.what()<<'\n';return 1;}
}
