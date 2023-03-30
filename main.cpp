#include <cassert>
#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include "digitalcurling3/digitalcurling3.hpp"

namespace dc = digitalcurling3;

namespace
{

    /// \brief ティーの位置
    constexpr dc::Vector2 kTee(
        dc::coordinate::GetCenterLineX(dc::coordinate::Id::kShot0),
        dc::coordinate::GetTeeLineY(true, dc::coordinate::Id::kShot0));

    /// \brief GameState::Stones のインデックス．
    struct StoneIndex
    {
        size_t team;
        size_t stone;
    };

    /// \brief ティー(ハウスの中心)からの距離でストーンをソートする．
    ///
    /// \param result 出力先パラメータ．ソート結果のインデックスが格納される．
    ///
    /// \param stones ストーンの配置
    void SortStones(std::array<StoneIndex, 16> &result, dc::GameState::Stones const &stones)
    {
        for (size_t i = 0; i < 16; ++i)
        {
            result[i].team = i / 8;
            result[i].stone = i % 8;
        }

        for (size_t i = 1; i < 16; ++i)
        {
            for (size_t j = i; j > 0; --j)
            {
                auto const &stone_a = stones[result[j - 1].team][result[j - 1].stone];
                auto const &stone_b = stones[result[j].team][result[j].stone];

                if (!stone_a || (stone_b && (stone_a->position - kTee).Length() > (stone_b->position - kTee).Length()))
                {
                    std::swap(result[j - 1], result[j]);
                }
            }
        }
    }

    /// \brief ストーンがハウス内にあるかを調べる．
    ///
    /// \return ストーンがハウス内にある場合 true ，そうでない場合 false
    bool IsInHouse(std::optional<dc::Transform> const &stone)
    {
        if (!stone)
            return false; // 盤面上にストーンが存在しない場合は false
        return (stone->position - kTee).Length() < dc::coordinate::kHouseRadius + dc::ISimulator::kStoneRadius;
    }

    /// \brief シミュレータFCV1において，指定地点を指定速度で通過するショットの初速を逆算します．
    ///
    /// 使用上の注意：
    /// - この関数はシミュレータFCV1に特化した関数です．他のシミュレータには対応していません．
    ///   したがって，この関数を使用した場合必然的にシミュレータFCV1に特化した思考エンジンになります．
    ///  （余談：複数のシミュレータに対応したショット速度を逆算する関数をライブラリから提供しさえすれば，
    ///   その関数を使用することで思考エンジンが特定のシミュレータに特化するといったことは無くなります．
    ///   ただ，今後どのようなシミュレータを追加するか現状では何とも言えなく，
    ///   どのようにショット速度を逆算すべきかも何とも言えないので，現状そのような関数はライブラリでは提供していません）
    /// - 関数内でシミュレータFCV1を使用して1ショット分シミュレーションを行っています．ですので，この関数の実行は高速とは言えません．
    /// - この関数は解析的にもとめたものでなく，シミュレーション結果から回帰分析で求めた関数です．したがって，特に飛距離にはある程度誤差が存在します．
    ///
    /// \param target_position 目標地点
    ///
    /// \param target_speed 目標地点到達時の速度．
    ///     0 にすればドローショット(石を停止させるショット)，0 より大きくすればヒットショット(石を他の石にぶつけるショット)になる．
    ///
    /// \param rotation ショットの回転方向
    ///
    /// \return 推測されたストーンの初速ベクトル
    dc::Vector2 EstimateShotVelocityFCV1(dc::Vector2 const &target_position, float target_speed, dc::moves::Shot::Rotation rotation)
    {
        assert(target_speed >= 0.f);
        assert(target_speed <= 4.f);

        // 初速度の大きさを逆算する
        // 逆算には専用の関数を用いる．

        float const v0_speed = [&target_position, target_speed]
        {
            auto const target_r = target_position.Length();
            assert(target_r > 0.f);

            if (target_speed <= 0.05f)
            {
                float constexpr kC0[] = {0.0005048122574925176, 0.2756242531609261};
                float constexpr kC1[] = {0.00046669575066030805, -29.898958358378636, -0.0014030973174948508};
                float constexpr kC2[] = {0.13968687866736632, 0.41120940058777616};

                float const c0 = kC0[0] * target_r + kC0[1];
                float const c1 = -kC1[0] * std::log(target_r + kC1[1]) + kC1[2];
                float const c2 = kC2[0] * target_r + kC2[1];

                return std::sqrt(c0 * target_speed * target_speed + c1 * target_speed + c2);
            }
            else if (target_speed <= 1.f)
            {
                float constexpr kC0[] = {-0.0014309170115803444, 0.9858457898438147};
                float constexpr kC1[] = {-0.0008339331735471273, -29.86751291726946, -0.19811799977982522};
                float constexpr kC2[] = {0.13967323742978, 0.42816312110477517};

                float const c0 = kC0[0] * target_r + kC0[1];
                float const c1 = -kC1[0] * std::log(target_r + kC1[1]) + kC1[2];
                float const c2 = kC2[0] * target_r + kC2[1];

                return std::sqrt(c0 * target_speed * target_speed + c1 * target_speed + c2);
            }
            else
            {
                float constexpr kC0[] = {1.0833113118071224e-06, -0.00012132851917870833, 0.004578093297561233, 0.9767006869364527};
                float constexpr kC1[] = {0.07950648211492622, -8.228225657195706, -0.05601306077702578};
                float constexpr kC2[] = {0.14140440186382008, 0.3875782508767419};

                float const c0 = kC0[0] * target_r * target_r * target_r + kC0[1] * target_r * target_r + kC0[2] * target_r + kC0[3];
                float const c1 = -kC1[0] * std::log(target_r + kC1[1]) + kC1[2];
                float const c2 = kC2[0] * target_r + kC2[1];

                return std::sqrt(c0 * target_speed * target_speed + c1 * target_speed + c2);
            }
        }();

        assert(target_speed < v0_speed);

        // 一度シミュレーションを行い，発射方向を決定する

        dc::Vector2 const delta = [rotation, v0_speed, target_speed]
        {
            float const rotation_factor = rotation == dc::moves::Shot::Rotation::kCCW ? 1.f : -1.f;

            // シミュレータは FCV1 シミュレータを使用する．
            thread_local std::unique_ptr<dc::ISimulator> s_simulator;
            if (s_simulator == nullptr)
            {
                s_simulator = dc::simulators::SimulatorFCV1Factory().CreateSimulator();
            }

            dc::ISimulator::AllStones init_stones;
            init_stones[0].emplace(dc::Vector2(), 0.f, dc::Vector2(0.f, v0_speed), 1.57f * rotation_factor);
            s_simulator->SetStones(init_stones);

            while (!s_simulator->AreAllStonesStopped())
            {
                auto const &stones = s_simulator->GetStones();
                auto const speed = stones[0]->linear_velocity.Length();
                if (speed <= target_speed)
                {
                    return stones[0]->position;
                }
                s_simulator->Step();
            }

            return s_simulator->GetStones()[0]->position;
        }();

        float const delta_angle = std::atan2(delta.x, delta.y); // 注: delta.x, delta.y の順番で良い
        float const target_angle = std::atan2(target_position.y, target_position.x);
        float const v0_angle = target_angle + delta_angle; // 発射方向

        return dc::Vector2(v0_speed * std::cos(v0_angle), v0_speed * std::sin(v0_angle));
    }

    // グローバル変数

    dc::Team g_team; /// 自分のチーム
    dc::GameSetting g_game_setting;
    std::unique_ptr<dc::ISimulator> g_simulator;
    std::unique_ptr<dc::ISimulatorStorage> g_simulator_storage;
    std::array<std::unique_ptr<dc::IPlayer>, 4> g_players;

    /// \brief サーバーから送られてきた試合設定が引数として渡されるので，試合前の準備を行います．
    ///
    /// 引数 \p player_order を編集することでプレイヤーのショット順を変更することができます．各プレイヤーの情報は \p player_factories に格納されています．
    /// 補足：プレイヤーはショットのブレをつかさどります．プレイヤー数は4で，0番目は0, 1投目，1番目は2, 3投目，2番目は4, 5投目，3番目は6, 7投目を担当します．
    ///
    /// この処理中の思考時間消費はありません．試合前に時間のかかる処理を行う場合この中で行うべきです．
    ///
    /// \param team この思考エンジンのチームID．
    ///     Team::k0 の場合，最初のエンドの先攻です．
    ///     Team::k1 の場合，最初のエンドの後攻です．
    ///
    /// \param game_setting 試合設定．
    ///     この参照はOnInitの呼出し後は無効になります．OnInitの呼出し後にも参照したい場合はコピーを作成してください．
    ///
    /// \param simulator_factory 試合で使用されるシミュレータの情報．
    ///     未対応のシミュレータの場合 nullptr が格納されます．
    ///
    /// \param player_factories 自チームのプレイヤー情報．
    ///     未対応のプレイヤーの場合 nullptr が格納されます．
    ///
    /// \param player_order 出力用引数．
    ///     プレイヤーの順番(デフォルトで0, 1, 2, 3)を変更したい場合は変更してください．
    void OnInit(
        dc::Team team,
        dc::GameSetting const &game_setting,
        std::unique_ptr<dc::ISimulatorFactory> simulator_factory,
        std::array<std::unique_ptr<dc::IPlayerFactory>, 4> player_factories,
        std::array<size_t, 4> &player_order)
    {
        // TODO AIを作る際はここを編集してくださいif (simulator_factory == nullptr || simulator_factory->GetSimulatorId() != "fcv1") {
        std::cout << "warning!: Unsupported simulator!"
            " EstimateShotVelocityFCV1() is only available for \"fcv1\" simulator." << std::endl;
        g_team = team;
        g_game_setting = game_setting;
        if (simulator_factory) {
            g_simulator = simulator_factory->CreateSimulator();
        }
        else {
            g_simulator = dc::simulators::SimulatorFCV1Factory().CreateSimulator();
        }
        g_simulator_storage = g_simulator->CreateStorage();

        // プレイヤーを生成する
        // 非対応の場合は NormalDistプレイヤーを使用する．
        assert(g_players.size() == player_factories.size());
        for (size_t i = 0; i < g_players.size(); ++i) {
            auto const& player_factory = player_factories[player_order[i]];
            if (player_factory) {
                g_players[i] = player_factory->CreatePlayer();
            }
            else {
                g_players[i] = dc::players::PlayerNormalDistFactory().CreatePlayer();
            }
        }
    }

    /// \brief 自チームのターンに呼ばれます．返り値として返した行動がサーバーに送信されます．
    ///
    /// \param game_state 現在の試合状況．
    ///     この参照は関数の呼出し後に無効になりますので，関数呼出し後に参照したい場合はコピーを作成してください．
    ///
    /// \return 選択する行動．この行動が自チームの行動としてサーバーに送信されます．
    dc::Move OnMyTurn(dc::GameState const &game_state)
    {
        using ShotRotation = dc::moves::Shot::Rotation;

        std::array<StoneIndex, 16> sorted_indices;
        SortStones(sorted_indices, game_state.stones);

        auto & current_player = *g_players[game_state.shot / 4];
        g_simulator->Save(*g_simulator_storage);
        for (auto const idx : sorted_indices)
        {
            if (idx.team == static_cast<size_t>(g_team))
                continue;
            auto const stone = game_state.stones[idx.team][idx.stone];
            if (!stone.has_value()) break;

            float speed = 0.5f;
            for (; speed < 3.5f; speed += 0.5f)
            {
                dc::GameState temp_game_state = game_state;
                dc::Move temp_move = dc::moves::Shot{ EstimateShotVelocityFCV1(stone->position, speed, ShotRotation::kCCW), ShotRotation::kCCW };
                g_simulator->Load(*g_simulator_storage);
                dc::ApplyMove(g_game_setting, *g_simulator, current_player, temp_game_state, temp_move, std::chrono::milliseconds(0));
                int shot = game_state.shot;
                if (temp_game_state.stones[shot % 2][shot / 2] && !temp_game_state.stones[idx.team][idx.stone])
                    break;
            }

            std::array<int, 2> points = { 0, 0 };
            std::array<dc::moves::Shot, 2> candidate_shots = {{
                {EstimateShotVelocityFCV1(stone->position, speed, ShotRotation::kCCW), ShotRotation::kCCW},
                {EstimateShotVelocityFCV1(stone->position, speed, ShotRotation::kCW), ShotRotation::kCW},
            }};
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < candidate_shots.size(); j++)
                {
                    g_simulator->Load(*g_simulator_storage);
                    dc::Move temp_move = candidate_shots[j];
                    dc::GameState temp_game_state = game_state;
                    dc::ApplyMove(g_game_setting, *g_simulator, current_player, temp_game_state, temp_move, std::chrono::milliseconds(0));
                    int shot = game_state.shot;
                    if (temp_game_state.stones[shot % 2][shot / 2].has_value())
                        points[j] += 1;
                    if (!temp_game_state.stones[idx.team][idx.stone].has_value())
                        points[j] += 1;
                }
            }
            if (points[0] > points[1]) return candidate_shots[0];
            else return candidate_shots[1];
        }

        auto const v0 = EstimateShotVelocityFCV1(kTee, 0.f, ShotRotation::kCCW);
        return dc::moves::Shot{v0, ShotRotation::kCCW};
    }

    /// \brief 相手チームのターンに呼ばれます．AIを作る際にこの関数の中身を記述する必要は無いかもしれません．
    ///
    /// ひとつ前の手番で自分が行った行動の結果を見ることができます．
    ///
    /// \param game_state 現在の試合状況．
    ///     この参照は関数の呼出し後に無効になりますので，関数呼出し後に参照したい場合はコピーを作成してください．
    void OnOpponentTurn(dc::GameState const &game_state)
    {
        // TODO AIを作る際はここを編集してください
    }

    /// \brief ゲームが正常に終了した際にはこの関数が呼ばれます．
    ///
    /// \param game_state 試合終了後の試合状況．
    ///     この参照は関数の呼出し後に無効になりますので，関数呼出し後に参照したい場合はコピーを作成してください．
    void OnGameOver(dc::GameState const &game_state)
    {
        // TODO AIを作る際はここを編集してください

        if (game_state.game_result->winner == g_team)
        {
            std::cout << "won the game" << std::endl;
        }
        else
        {
            std::cout << "lost the game" << std::endl;
        }
    }

} // unnamed namespace

int main(int argc, char const *argv[])
{
    using boost::asio::ip::tcp;
    using nlohmann::json;

    // TODO AIの名前を変更する場合はここを変更してください．
    constexpr auto kName = "AIcyObsidian";

    constexpr int kSupportedProtocolVersionMajor = 1;

    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: command <host> <port>" << std::endl;
            return 1;
        }

        boost::asio::io_context io_context;

        tcp::socket socket(io_context);
        tcp::resolver resolver(io_context);
        boost::asio::connect(socket, resolver.resolve(argv[1], argv[2])); // 引数のホスト，ポートに接続します．

        // ソケットから1行読む関数です．バッファが空の場合，新しい行が来るまでスレッドをブロックします．
        auto read_next_line = [&socket, input_buffer = std::string()]() mutable
        {
            // read_untilの結果，input_bufferに複数行入ることがあるため，1行ずつ取り出す処理を行っている
            if (input_buffer.empty())
            {
                boost::asio::read_until(socket, boost::asio::dynamic_buffer(input_buffer), '\n');
            }
            auto new_line_pos = input_buffer.find_first_of('\n');
            auto line = input_buffer.substr(0, new_line_pos + 1);
            input_buffer.erase(0, new_line_pos + 1);
            return line;
        };

        // コマンドが予期したものかチェックする関数です．
        auto check_command = [](nlohmann::json const &jin, std::string_view expected_cmd)
        {
            auto const actual_cmd = jin.at("cmd").get<std::string>();
            if (actual_cmd != expected_cmd)
            {
                std::ostringstream buf;
                buf << "Unexpected cmd (expected: \"" << expected_cmd << "\", actual: \"" << actual_cmd << "\")";
                throw std::runtime_error(buf.str());
            }
        };

        dc::Team team = dc::Team::kInvalid;

        // [in] dc
        {
            auto const line = read_next_line();
            auto const jin = json::parse(line);

            check_command(jin, "dc");

            auto const &jin_version = jin.at("version");
            if (jin_version.at("major").get<int>() != kSupportedProtocolVersionMajor)
            {
                throw std::runtime_error("Unexpected protocol version");
            }

            std::cout << "[in] dc" << std::endl;
            std::cout << "  game_id  : " << jin.at("game_id").get<std::string>() << std::endl;
            std::cout << "  date_time: " << jin.at("date_time").get<std::string>() << std::endl;
        }

        // [out] dc_ok
        {
            json const jout = {
                {"cmd", "dc_ok"},
                {"name", kName}};
            auto const output_message = jout.dump() + '\n';
            boost::asio::write(socket, boost::asio::buffer(output_message));

            std::cout << "[out] dc_ok" << std::endl;
            std::cout << "  name: " << kName << std::endl;
        }

        // [in] is_ready
        {
            auto const line = read_next_line();
            auto const jin = json::parse(line);

            check_command(jin, "is_ready");

            if (jin.at("game").at("rule").get<std::string>() != "normal")
            {
                throw std::runtime_error("Unexpected rule");
            }

            team = jin.at("team").get<dc::Team>();

            auto const game_setting = jin.at("game").at("setting").get<dc::GameSetting>();

            auto const &jin_simulator = jin.at("game").at("simulator");
            std::unique_ptr<dc::ISimulatorFactory> simulator_factory;
            try
            {
                simulator_factory = jin_simulator.get<std::unique_ptr<dc::ISimulatorFactory>>();
            }
            catch (std::exception &e)
            {
                std::cout << "Exception: " << e.what() << std::endl;
            }

            auto const &jin_player_factories = jin.at("game").at("players").at(dc::ToString(team));
            std::array<std::unique_ptr<dc::IPlayerFactory>, 4> player_factories;
            for (size_t i = 0; i < 4; ++i)
            {
                std::unique_ptr<dc::IPlayerFactory> player_factory;
                try
                {
                    player_factory = jin_player_factories[i].get<std::unique_ptr<dc::IPlayerFactory>>();
                }
                catch (std::exception &e)
                {
                    std::cout << "Exception: " << e.what() << std::endl;
                }
                player_factories[i] = std::move(player_factory);
            }

            std::cout << "[in] is_ready" << std::endl;

            // [out] ready_ok

            std::array<size_t, 4> player_order{0, 1, 2, 3};
            OnInit(team, game_setting, std::move(simulator_factory), std::move(player_factories), player_order);

            json const jout = {
                {"cmd", "ready_ok"},
                {"player_order", player_order}};
            auto const output_message = jout.dump() + '\n';
            boost::asio::write(socket, boost::asio::buffer(output_message));

            std::cout << "[out] ready_ok" << std::endl;
            std::cout << "  player order: " << jout.at("player_order").dump() << std::endl;
        }

        // [in] new_game
        {
            auto const line = read_next_line();
            auto const jin = json::parse(line);

            check_command(jin, "new_game");

            std::cout << "[in] new_game" << std::endl;
            std::cout << "  team 0: " << jin.at("name").at("team0") << std::endl;
            std::cout << "  team 1: " << jin.at("name").at("team1") << std::endl;
        }

        dc::GameState game_state;

        while (true)
        {
            // [in] update
            auto const line = read_next_line();
            auto const jin = json::parse(line);

            check_command(jin, "update");

            game_state = jin.at("state").get<dc::GameState>();

            std::cout << "[in] update (end: " << int(game_state.end) << ", shot: " << int(game_state.shot) << ")" << std::endl;

            // if game was over
            if (game_state.game_result)
            {
                break;
            }

            if (game_state.GetNextTeam() == team)
            { // my turn
                // [out] move
                auto move = OnMyTurn(game_state);
                json jout = {
                    {"cmd", "move"},
                    {"move", move}};
                auto const output_message = jout.dump() + '\n';
                boost::asio::write(socket, boost::asio::buffer(output_message));

                std::cout << "[out] move" << std::endl;
                if (std::holds_alternative<dc::moves::Shot>(move))
                {
                    dc::moves::Shot const &shot = std::get<dc::moves::Shot>(move);
                    std::cout << "  type    : shot" << std::endl;
                    std::cout << "  velocity: [" << shot.velocity.x << ", " << shot.velocity.y << "]" << std::endl;
                    std::cout << "  rotation: " << (shot.rotation == dc::moves::Shot::Rotation::kCCW ? "ccw" : "cw") << std::endl;
                }
                else if (std::holds_alternative<dc::moves::Concede>(move))
                {
                    std::cout << "  type: concede" << std::endl;
                }
            }
            else
            { // opponent turn
                OnOpponentTurn(game_state);
            }
        }

        // [in] game_over
        {
            auto const line = read_next_line();
            auto const jin = json::parse(line);

            check_command(jin, "game_over");

            std::cout << "[in] game_over" << std::endl;
        }

        // 終了．
        OnGameOver(game_state);
    }
    catch (std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception" << std::endl;
    }

    return 0;
}