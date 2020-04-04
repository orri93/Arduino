#ifndef GOS_PID_TOOLKIT_VERSION_H_
#define GOS_PID_TOOLKIT_VERSION_H_

/* The major version, (1, if %PROJECT_VERSION is 1.2.3.4) */
#define GPT_VERSION_MAJOR (@PROJECT_VERSION_MAJOR@)

/* The minor version (2, if %PROJECT_VERSION is 1.2.3.4) */
#define GPT_VERSION_MINOR (@PROJECT_VERSION_MINOR@)

/* The patch version (3, if %PROJECT_VERSION is 1.2.3.4) */
#define GPT_VERSION_PATCH (@PROJECT_VERSION_PATCH@)

/* The build version (4, if %PROJECT_VERSION is 1.2.3.4) */
#define GPT_VERSION_BUILD (@PROJECT_VERSION_TWEAK@)

/* The full version, like 1.2.3.4 */
#define GPT_VERSION         @PROJECT_VERSION@

/* The full version, in string form (suited for string concatenation) */
#define GPT_VERSION_STRING "@PROJECT_VERSION@"

namespace gos {
namespace pid {
namespace toolkit {
namespace version {

typedef unsigned char Option;

extern const Option WithAll;
extern const Option WithPrefix;
extern const Option WithBuildDateTime;

std::string generate(const Option& option = WithPrefix);

} // namespace version
} // namespace toolkit
} // namespace pid
} // namespace gos

#endif
